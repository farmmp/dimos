# Copyright 2026 Dimensional Inc.
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

"""Tests for the Gazebo native simulation module.

Markers:
    - Unit tests (config / port introspection) run on any platform.
    - Integration test that launches the real gz-sim+bridge:
        @pytest.mark.slow
        Skipped automatically if the gazebo_native binary is not built or
        the gz-sim binary is not resolvable via the cpp/ flake.
"""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import time

import pytest

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.gazebo.module import Gazebo, GazeboConfig

_PKG = Path(__file__).resolve().parent
_CPP_DIR = _PKG / "cpp"
_BRIDGE_RESULT = _CPP_DIR / "result" / "bin" / "gazebo_native"


def _nix_path(attr: str) -> str | None:
    """Return the store path for a flake output, or None if not built."""
    if shutil.which("nix") is None:
        return None
    try:
        out = subprocess.run(
            ["nix", "path-info", f".#{attr}"],
            cwd=str(_CPP_DIR),
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    p = out.stdout.strip().splitlines()
    return p[0] if p and Path(p[0]).exists() else None


def _resolve_bridge_binary() -> str | None:
    """Find the gazebo_native bridge binary."""
    if _BRIDGE_RESULT.exists():
        return str(_BRIDGE_RESULT)
    base = _nix_path("gazebo_native")
    if not base:
        return None
    candidate = Path(base) / "bin" / "gazebo_native"
    return str(candidate) if candidate.exists() else None


# ---------------------------------------------------------------------------
# Always-on unit tests
# ---------------------------------------------------------------------------


class TestConfig:
    def test_defaults(self) -> None:
        cfg = GazeboConfig()
        assert cfg.executable == "result/bin/gazebo_native"
        assert cfg.cwd == "cpp"
        assert cfg.headless is True
        assert cfg.gz_cmd_vel == "/model/dimos_bot/cmd_vel"
        assert cfg.gz_odom == "/model/dimos_bot/odometry"
        assert cfg.gz_lidar == "/lidar/points"
        # Default world ships in the package
        assert Path(cfg.world).name == "dimos_bot.sdf"
        assert Path(cfg.world).exists(), "bundled world file should exist"

    def test_overrides(self) -> None:
        cfg = GazeboConfig(headless=False, gz_cmd_vel="/foo/cmd")
        assert cfg.headless is False
        assert cfg.gz_cmd_vel == "/foo/cmd"


class TestPorts:
    """Verify the Gazebo module declares the same I/O contract as Unity."""

    REQUIRED_IN = {"cmd_vel": Twist, "terrain_map": PointCloud2}
    REQUIRED_OUT = {
        "odometry": Odometry,
        "registered_scan": PointCloud2,
        "color_image": Image,
        "semantic_image": Image,
        "camera_info": CameraInfo,
    }

    def test_module_declares_unity_io_contract(self) -> None:
        # Pydantic class-level annotations describe declared ports.
        anns = Gazebo.__annotations__
        for name, _ty in {**self.REQUIRED_IN, **self.REQUIRED_OUT}.items():
            assert name in anns, f"port {name!r} missing on Gazebo module"

    def test_topics_collected_for_all_ports(self) -> None:
        # Instantiate locally (no coordinator) to reach _collect_topics().
        mod = Gazebo()
        transports: list[LCMTransport] = []
        try:
            for name, ty in self.REQUIRED_OUT.items():
                t = LCMTransport(f"/test/{name}", ty)
                transports.append(t)
                getattr(mod, name).transport = t
            for name, ty in self.REQUIRED_IN.items():
                t = LCMTransport(f"/test/{name}", ty)
                transports.append(t)
                getattr(mod, name).transport = t
            topics = mod._collect_topics()
            for name in {**self.REQUIRED_IN, **self.REQUIRED_OUT}:
                assert name in topics, f"NativeModule did not collect topic for {name!r}"
            # Verify the auto-generated CLI args carry our extra config knobs.
            cli = mod.config.to_cli_args()
            assert "--world" in cli
            assert "--gz_cmd_vel" in cli
        finally:
            for t in transports:
                t.stop()
            mod.stop()  # tears down the asyncio loop thread spawned in __init__


# ---------------------------------------------------------------------------
# Integration test
# ---------------------------------------------------------------------------


_BRIDGE = _resolve_bridge_binary()


@pytest.mark.slow
@pytest.mark.skipif(_BRIDGE is None, reason="gazebo_native bridge not built (run nix build .#gazebo_native in cpp/)")
class TestLiveGazebo:
    """Full end-to-end: launch the bridge (with embedded gz::sim::Server),
    send a cmd_vel, expect odometry / scan / color_image to flow back."""

    def test_sim_publishes_streams(self) -> None:
        # Run on a unique GZ partition so concurrent tests can't cross-talk.
        partition = f"dimos_test_{os.getpid()}"
        os.environ["GZ_PARTITION"] = partition

        # Instantiate the module directly (no coordinator/worker indirection)
        # so we can deterministically stop every LCM transport at teardown.
        mod = Gazebo(
            executable=_BRIDGE,           # absolute path; bypass cwd lookup
            cwd=None,
            build_command=None,           # already built
            headless=True,
            # Full sensor world — uses gz-rendering 9 ogre2 engine via
            # OGRE-Next 2.3 with EGL surfaceless. Cameras + lidar render
            # via mesa swrast/llvmpipe; no DISPLAY / X server required.
            world=str(_PKG / "worlds" / "dimos_bot.sdf"),
        )
        transports_to_stop: list = []
        try:
            # Wire each port to a unique LCM topic so we can introspect.
            for name, ty in (
                ("cmd_vel",         Twist),
                ("terrain_map",     PointCloud2),
                ("odometry",        Odometry),
                ("registered_scan", PointCloud2),
                ("color_image",     Image),
                ("camera_info",     CameraInfo),
                ("semantic_image",  Image),
            ):
                t = LCMTransport(f"/{partition}/{name}", ty)
                getattr(mod, name).transport = t
                transports_to_stop.append(t)

            # Counters incremented via subscribers on the OUT ports.
            counts = {"odometry": 0, "registered_scan": 0, "color_image": 0}

            def _bump(name: str):
                def cb(_msg) -> None:
                    counts[name] += 1
                return cb

            # Subscribe via the LCM transport directly — `mod.<port>.subscribe`
            # only chains a Stream callback, but the bridge publishes via LCM.
            mod.odometry.transport.subscribe(_bump("odometry"))
            mod.registered_scan.transport.subscribe(_bump("registered_scan"))
            mod.color_image.transport.subscribe(_bump("color_image"))

            # Snapshot the transports we created so we can stop them cleanly.
            transports_to_stop.extend(
                getattr(mod, name).transport
                for name in ("cmd_vel", "terrain_map", "odometry",
                             "registered_scan", "color_image", "camera_info",
                             "semantic_image")
            )

            mod.start()

            # Drive forward so DiffDrive integrates and odometry ticks.
            # Publish directly on the cmd_vel LCM channel since `cmd_vel` is
            # an In port on the module (not directly publishable from here).
            time.sleep(3.0)  # gz-sim spawn time
            mod.cmd_vel.transport.broadcast(
                None,
                Twist(linear=Vector3(x=0.3, y=0.0, z=0.0),
                      angular=Vector3(x=0.0, y=0.0, z=0.0)),
            )

            # Wait up to 60s for at least one of each stream.
            # Software-rendered EGL is slow on first paint; cameras /
            # lidar take longer than odometry to start ticking.
            deadline = time.time() + 60.0
            while time.time() < deadline:
                if all(c > 0 for c in counts.values()):
                    break
                time.sleep(0.5)

            mod.stop()
        finally:
            for t in transports_to_stop:
                try:
                    t.stop()
                except Exception:
                    pass
            try:
                mod.stop()
            except Exception:
                pass
            os.environ.pop("GZ_PARTITION", None)

        assert counts["odometry"] > 0, (
            "no odometry messages received — bridge did not relay gz-sim's "
            "odometry to LCM (round trip cmd_vel→sim→odom→bridge→LCM broken)"
        )
        assert counts["registered_scan"] > 0, (
            "no lidar PointCloud2 messages — gpu_lidar sensor isn't producing "
            "frames headlessly. Check OGRE-Next EGL surfaceless setup."
        )
        assert counts["color_image"] > 0, (
            "no color image messages — RGB camera sensor isn't producing "
            "frames headlessly. Check OGRE-Next EGL surfaceless setup."
        )
