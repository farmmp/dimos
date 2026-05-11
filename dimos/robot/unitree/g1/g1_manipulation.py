# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""G1-aware ManipulationModule that keeps Drake's pelvis aligned with /odom.

The G1 catalog uses ``weld_base=False`` so Drake treats the pelvis as a
6-DOF floating body.  Before each Cartesian plan we push the latest
``/odom`` pose into Drake via ``WorldSpec.set_floating_base_pose`` —
that way Drake's world frame matches MuJoCo's world frame and the
parent ``move_to_pose`` / ``pick`` / ``refresh_obstacles`` paths can use
world coordinates throughout (no per-skill frame conversions).
"""

from __future__ import annotations

import threading
from typing import Any, Literal

import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.pointing import solve_pointing
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1ManipulationModule(PickAndPlaceModule):
    """PickAndPlaceModule that syncs Drake's floating-base pelvis to /odom.

    All Cartesian skills inherited from PickAndPlaceModule (move_to_pose,
    pick, place, drop_on, refresh_obstacles, look, scan_objects, …) work
    unmodified — they all consume world-frame coordinates and Drake's
    plant has the pelvis welded at the live /odom pose for the duration
    of each plan.
    """

    odom: In[PoseStamped]

    _latest_odom: PoseStamped | None
    _odom_lock: threading.Lock

    def __init__(self, *, sim_mjcf_path: str | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_odom = None
        self._odom_lock = threading.Lock()
        # Easy-mode "ground truth" object lookup: when set, lets the
        # reach_for_sim_object skill pull body world poses straight from
        # the MJCF instead of going through perception.  Lazy-loaded.
        self._sim_mjcf_path = sim_mjcf_path
        self._sim_model: Any = None
        self._sim_data: Any = None

    @rpc
    def start(self) -> None:
        super().start()
        try:
            unsub = self.odom.subscribe(self._on_odom)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"G1ManipulationModule: odom subscribe failed: {e}")

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._odom_lock:
            self._latest_odom = msg
        # Sync pelvis into Drake on every odom tick so get_ee_pose
        # (used by go_init's safe-waypoint and any external query)
        # sees a live pose.  The set_floating_base_pose itself is
        # cheap; what was killing IK was the meshcat publish it
        # used to trigger — that's now gone (visualization will
        # update on the next plan).
        self._sync_floating_base()

    def _get_default_robot_name(self) -> str | None:
        # Base picks "single registered robot" else None — which makes every
        # skill fail with "Multiple robots configured" the moment two arms
        # are registered.  Prefer left_arm when both are present so the LLM
        # can call point_at / scan_objects / go_home etc. without having to
        # know about robot_name.  move_to_pose / point_at do their own
        # smarter target-side picking before this fallback runs.
        if "left_arm" in self._robots:
            return "left_arm"
        return super()._get_default_robot_name()

    def _begin_planning(self, robot_name: Any = None) -> Any:
        self._sync_floating_base()
        return super()._begin_planning(robot_name)

    def _sync_floating_base(self) -> None:
        if self._world_monitor is None:
            return
        with self._odom_lock:
            odom = self._latest_odom
        if odom is None:
            return
        world = self._world_monitor.world
        setter = getattr(world, "set_floating_base_pose", None)
        if setter is None:
            return
        for robot_name, (robot_id, _, _) in self._robots.items():
            try:
                setter(robot_id, odom)
            except Exception as e:
                logger.debug(f"set_floating_base_pose failed for {robot_name}: {e}")

    # ------------------------------------------------------------------
    # Easy mode: bypass perception, use MJCF ground-truth body positions
    # ------------------------------------------------------------------
    def _ensure_sim_model(self) -> bool:
        """Lazy-load the MJCF model used for ground-truth lookups."""
        if self._sim_model is not None:
            return True
        if not self._sim_mjcf_path:
            return False
        try:
            import mujoco
        except ImportError:
            logger.warning("mujoco not installed; reach_for_sim_object disabled")
            return False
        try:
            # The G1 MJCF references mesh STL/OBJs by bare filename
            # (Menagerie convention).  MujocoSimModule injects the
            # bytes via dimos.simulation.mujoco.model.get_assets — do
            # the same here so from_xml_string can find them without
            # depending on the working directory.
            from dimos.simulation.mujoco.model import get_assets

            assets = get_assets()
            with open(self._sim_mjcf_path) as f:
                xml_str = f.read()
            self._sim_model = mujoco.MjModel.from_xml_string(xml_str, assets=assets)
            self._sim_data = mujoco.MjData(self._sim_model)
            mujoco.mj_forward(self._sim_model, self._sim_data)
            logger.info(f"Sim ground-truth model loaded from {self._sim_mjcf_path}")
            return True
        except Exception as e:
            logger.warning(f"Failed to load sim model: {e}")
            return False

    @skill
    def point_at(
        self,
        x: float,
        y: float,
        z: float,
        robot_name: str | None = None,
    ) -> str:
        """Aim the closer arm so the fingertip points at a world point.

        Closed-form heuristic — picks left or right arm based on which
        side of the body the target is on, then solves shoulder pitch/
        roll directly. Far faster and more predictable than IK; sub-ms
        compute, deterministic poses, no random restarts.

        For 6-DOF EE pose tracking (grasping etc.), use ``move_to_pose``.

        Args:
            x: Target X position in meters (world frame).
            y: Target Y position in meters (world frame).
            z: Target Z position in meters (world frame).
            robot_name: Force a specific arm. Default is auto-select.
        """
        with self._odom_lock:
            pelvis = self._latest_odom
        if pelvis is None:
            return "Error: no /odom yet — robot pose unknown"

        target = np.array([x, y, z], dtype=np.float64)
        side: Literal["left", "right", "auto"] = "auto"
        if robot_name == "left_arm":
            side = "left"
        elif robot_name == "right_arm":
            side = "right"

        sol = solve_pointing(target, pelvis, side=side)
        if sol is None:
            return (
                f"Error: ({x:.2f}, {y:.2f}, {z:.2f}) is outside the arm's "
                f"pointing workspace (behind torso, or too far above the shoulder)."
            )

        chosen_robot = "left_arm" if sol.side == "left" else "right_arm"
        if chosen_robot not in self._robots:
            return f"Error: '{chosen_robot}' not registered"
        robot_id, config, _ = self._robots[chosen_robot]

        # Read current arm joints in URDF order via the planning world.
        if self._world_monitor is None:
            return "Error: planning world not initialized"
        world = self._world_monitor.world
        with world.scratch_context() as ctx:
            seed = world.get_joint_state(ctx, robot_id)
        q_start = np.array(seed.position, dtype=np.float64)
        q_target = np.array([sol.joints[name] for name in config.joint_names], dtype=np.float64)

        traj = self._build_arm_trajectory(config.joint_names, q_start, q_target)

        client = self._get_coordinator_client()
        if client is None or not config.coordinator_task_name:
            return "Error: coordinator client unavailable"
        translated = self._translate_trajectory_to_coordinator(traj, config)
        accepted = client.task_invoke(
            config.coordinator_task_name, "execute", {"trajectory": translated}
        )
        if not accepted:
            return "Error: coordinator rejected pointing trajectory"

        if not self._wait_for_trajectory_completion(chosen_robot, timeout=8.0):
            return "Error: pointing trajectory timed out"

        return f"Pointing {sol.side} arm at ({x:.2f}, {y:.2f}, {z:.2f})"

    @staticmethod
    def _build_arm_trajectory(
        joint_names: list[str],
        q_start: np.ndarray,
        q_target: np.ndarray,
        duration: float = 2.5,
        n_waypoints: int = 9,
        via_zero: bool = True,
    ) -> JointTrajectory:
        """Cosine-smoothed JointTrajectory from q_start to q_target.

        With ``via_zero=True`` (default), the trajectory passes through
        the all-zeros pose at the midpoint — a forced reset between
        successive ``point_at`` calls so each pointing starts from the
        same baseline. Without this, repeated point_at calls compound
        residual joint errors.
        """
        if via_zero:
            q_mid = np.zeros_like(q_start)
            half_n = (n_waypoints + 1) // 2
            half_dur = duration / 2.0
            points: list[TrajectoryPoint] = []
            # Phase 1: q_start -> 0 (cosine-smoothed)
            for i in range(half_n):
                s = i / (half_n - 1) if half_n > 1 else 0.0
                alpha = 0.5 - 0.5 * float(np.cos(np.pi * s))
                q = q_start + alpha * (q_mid - q_start)
                points.append(TrajectoryPoint(positions=q.tolist(), time_from_start=s * half_dur))
            # Phase 2: 0 -> q_target (cosine-smoothed); skip first point
            # to avoid duplicating the zero waypoint at the seam.
            for i in range(1, n_waypoints - half_n + 1):
                s = i / (n_waypoints - half_n)
                alpha = 0.5 - 0.5 * float(np.cos(np.pi * s))
                q = q_mid + alpha * (q_target - q_mid)
                points.append(
                    TrajectoryPoint(
                        positions=q.tolist(),
                        time_from_start=half_dur + s * half_dur,
                    )
                )
            return JointTrajectory(joint_names=list(joint_names), points=points)

        points = []
        for i in range(n_waypoints):
            s = i / (n_waypoints - 1)
            alpha = 0.5 - 0.5 * float(np.cos(np.pi * s))
            q = q_start + alpha * (q_target - q_start)
            points.append(TrajectoryPoint(positions=q.tolist(), time_from_start=s * duration))
        return JointTrajectory(joint_names=list(joint_names), points=points)

    @skill
    def point_at_sim_object(
        self,
        body_name: str = "manip_cube",
        robot_name: str | None = None,
    ) -> str:
        """Easy-mode: point the arm at a sim object using its MJCF ground-truth pose.

        Same MJCF-bypass-perception approach as ``reach_for_sim_object``,
        but uses the (out-of-reach-tolerant) ``point_at`` skill instead
        of trying to grasp.  Useful for verifying the arm-aiming pipeline
        when the object is too far for the arm to reach.

        Args:
            body_name: MJCF body to point at (default 'manip_cube').
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        if not self._ensure_sim_model():
            return "Easy mode unavailable: sim_mjcf_path not configured."
        import mujoco

        body_id = mujoco.mj_name2id(self._sim_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return f"Body '{body_name}' not found in MJCF."
        pos = self._sim_data.xpos[body_id]
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        logger.info(f"point_at_sim_object('{body_name}') → world ({x:.3f}, {y:.3f}, {z:.3f})")
        return self.point_at(x=x, y=y, z=z, robot_name=robot_name)

    @skill
    def reach_for_sim_object(
        self,
        body_name: str = "manip_cube",
        robot_name: str | None = None,
    ) -> str:
        """Easy-mode: reach for a sim object using its MJCF ground-truth pose.

        Bypasses the perception pipeline (YOLO-E detection, RGBD
        back-projection, frame transforms) and instead reads the
        target body's world pose directly from the MuJoCo model.  Use
        this to isolate manipulation issues from perception issues —
        if this works but ``move_to_pose`` after ``detect`` doesn't,
        the bug is in perception.

        Args:
            body_name: MJCF body to reach for (default 'manip_cube').
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        if not self._ensure_sim_model():
            return "Easy mode unavailable: sim_mjcf_path not configured."
        import mujoco

        body_id = mujoco.mj_name2id(self._sim_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return f"Body '{body_name}' not found in MJCF."
        pos = self._sim_data.xpos[body_id]
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        logger.info(f"reach_for_sim_object('{body_name}') → world ({x:.3f}, {y:.3f}, {z:.3f})")
        return self.move_to_pose(x=x, y=y, z=z, robot_name=robot_name)
