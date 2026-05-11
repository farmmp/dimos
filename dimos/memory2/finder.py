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

"""Production version of the ``test_detect_objects_smart`` recipe.

Single-module variant that owns the memory2 SQLite store, embeds
images live with CLIP, and exposes ``find_object_3d(label)`` — runs
the chain ``.search → .near → QualityWindow → .map(vlm) →
Detection3DPC.from_2d`` to turn "find me a bottle" into a
world-frame 3D position.

Why one module instead of three (Recorder + SemanticSearch +
ObjectFinder3D): each ``MemoryModule`` instantiates its own
``SqliteStore`` with its own in-process ``SubjectNotifier``. Even
when they share a ``db_path``, ``stream.live()`` on one instance
doesn't see ``stream.append()`` on another — the notifier is
per-instance, not per-file. Collapsing the three concerns into one
module gives them one store, one notifier, and a working live
pipeline. (This is also why Lesh's ``unitree_go2_memory`` ships
only ``Recorder`` and runs ``SemanticSearch`` against pre-imported
recordings — live embedding wasn't unblocked yet.)
"""

from __future__ import annotations

import threading
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.memory2.embed import EmbedImages
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.memory2.transform import QualityWindow
from dimos.models.embedding.base import EmbeddingModel
from dimos.models.embedding.clip import CLIPModel
from dimos.models.vl.base import VlModel
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.visualization_msgs.BBoxMarkers import BBoxMarker, BBoxMarkers
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.protocol.tf.tf import LCMTF
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ObjectFinder3DConfig(RecorderConfig):
    """Config for ``ObjectFinder3D``."""

    embedding_model: type[EmbeddingModel] = CLIPModel
    vlm_model: type[VlModel] = MoondreamVlModel
    # Spatial+temporal window around the semantic hotspot.
    near_radius_m: float = 1.0
    near_tolerance_s: float = 60.0
    quality_window_s: float = 1.0
    # Live embedding filters (mirrors SemanticSearch defaults).
    min_brightness: float = 0.1
    embedding_quality_window_s: float = 0.5
    embedding_batch_size: int = 2
    # TF buffer length — the default LCMTF buffer keeps only 10s of
    # transforms, but we look up TFs by *recorded image* timestamps
    # which can be minutes old. Bump to 1 hour by default.
    tf_buffer_size_s: float = 3600.0


class ObjectFinder3D(Recorder):
    """Records, embeds, and finds — all backed by one SQLite store.

    Subscribes:
      * ``color_image: In[Image]`` — raw RGB, recorded with attached pose
      * ``lidar: In[PointCloud2]`` — pointcloud, recorded with attached pose
      * ``odom: In[PoseStamped]`` — pose source for the above two
      * ``camera_info: In[CameraInfo]`` — optical-frame intrinsics, cached

    On start, kicks off a live CLIP embedding pipeline that writes the
    ``color_image_embedded`` stream into the same store.

    Skill ``find_object_3d(label)`` runs the test_visualizer recipe
    end-to-end and returns a string with the world-frame xyz of the
    first matching detection.
    """

    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    camera_info: In[CameraInfo]
    # Snapshot of the most recent find_object_3d result, for live
    # overlay in viser. Each successful find replaces this snapshot
    # so the visualizer always shows where we last looked.
    found_objects: Out[BBoxMarkers]
    # The subset of lidar points that ended up inside the bbox
    # (i.e., the points whose centroid is the published bbox center).
    # Visualizing this in viser tells you whether back-projection
    # selected the right cluster or e.g. landed on a wall behind
    # the actual object.
    found_pointcloud: Out[PointCloud2]
    config: ObjectFinder3DConfig

    _embedder: EmbeddingModel | None = None
    _vlm: VlModel | None = None
    _embeddings: Any = None  # memory2.Stream[Image]; Any to avoid forward-ref evaluation
    _camera_info: CameraInfo | None = None
    _camera_info_lock: threading.Lock
    # Accumulated bbox snapshot — each find_object_3d adds/updates one
    # entry by label; previous entries persist so all found objects
    # stay visible in viser at once. Use clear_found_objects() to reset.
    _found: dict[str, BBoxMarker]
    _found_lock: threading.Lock

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._camera_info_lock = threading.Lock()
        self._found = {}
        self._found_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        # Recorder.start() walks self.inputs and pipes each port into
        # the store with pose attached (for non-pose ports). It also
        # records camera_info, which is harmless — we additionally
        # cache the latest in-memory below for back-projection.
        super().start()

        # Activate TF reception. Splat camera publishes its
        # world-to-optical transform under (frame_id="world",
        # child_frame_id=image.frame_id) on every render tick — that's
        # the correct extrinsics for Detection3DPC.from_2d, since the
        # `pose` attached to recorded images is the pelvis (/odom),
        # not the head-mounted camera optical frame.
        #
        # Override the default TF buffer (10s) before the lazy property
        # creates the default instance — we look up TFs by *recorded*
        # image timestamps that can be many minutes old.
        self._tf = LCMTF(buffer_size=self.config.tf_buffer_size_s)
        self.tf.start()

        # Cache camera_info for find_object_3d (stream lookup at find
        # time would also work, but in-memory cache is simpler).
        try:
            unsub = self.camera_info.subscribe(self._on_camera_info)
            self.register_disposable(Disposable(unsub))
        except Exception as e:
            logger.warning(f"ObjectFinder3D: camera_info subscribe failed: {e}")

        # CLIP + VLM lazy-load on the worker process (one-time GPU/disk cost).
        self._embedder = self.register_disposable(self.config.embedding_model())
        self._embedder.start()
        self._vlm = self.register_disposable(self.config.vlm_model())
        self._vlm.start()

        # Live embedding pipeline — same shape as SemanticSearch.start
        # but on our own store so .live() actually receives the appends
        # Recorder.start() just wired up.
        self._embeddings = self.store.stream("color_image_embedded", Image)
        # fmt: off
        sub = (
            self.store.streams.color_image
            .live()
            .filter(lambda obs: obs.data.brightness > self.config.min_brightness)
            .transform(QualityWindow(lambda img: img.sharpness, window=self.config.embedding_quality_window_s))
            .transform(EmbedImages(self._embedder, batch_size=self.config.embedding_batch_size))
            .save(self._embeddings)
            .drain_thread()
        )
        # fmt: on
        self.register_disposable(Disposable(sub.dispose))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        with self._camera_info_lock:
            self._camera_info = msg

    @skill
    def clear_found_objects(self) -> str:
        """Wipe the accumulated bbox snapshot in viser. Useful between tests."""
        with self._found_lock:
            self._found.clear()
        self.found_objects.publish(BBoxMarkers(markers=[]))
        return "Cleared all found-object bboxes."

    @skill
    def find_object_3d(self, label: str) -> str:
        """Search memory for ``label``; return its world-frame xyz.

        Returns ``Found '{label}' at (x, y, z)`` on success, or a
        diagnostic message on miss.
        """
        if self._embedder is None or self._vlm is None or self._embeddings is None:
            return "Error: ObjectFinder3D not started"
        with self._camera_info_lock:
            cam_info = self._camera_info
        if cam_info is None:
            return "Error: no camera_info received yet"

        store = self.store
        embedded = self._embeddings
        lidar = store.streams.lidar

        # Walk top-k semantic hits; the early ones may be from before
        # odom started (pose_stamped raises LookupError). Skip those
        # and try the next.
        hotspot = None
        for candidate in embedded.search(self._embedder.embed_text(label), k=10):
            try:
                _ = candidate.pose_stamped
            except LookupError:
                continue
            hotspot = candidate
            break
        if hotspot is None:
            return f"No pose-attached memory of '{label}' yet — drive around to record more."

        n_candidate_frames = 0
        n_dets = 0
        n_tf_misses = 0
        for obs in (
            store.streams.color_image.at(
                hotspot.pose_stamped.ts, tolerance=self.config.near_tolerance_s
            )
            .near(hotspot.pose_stamped, radius=self.config.near_radius_m)
            .transform(
                QualityWindow(lambda img: img.sharpness, window=self.config.quality_window_s)
            )
            .map(lambda o: o.derive(data=self._vlm.query_detections(o.data, label)))  # type: ignore[union-attr]
        ):
            n_candidate_frames += 1
            try:
                _ = obs.pose_stamped  # skip frames that lost pose to the startup race
            except LookupError:
                continue
            try:
                lidar_obs = lidar.at(obs.ts).first()
            except LookupError:
                continue
            # Look up the camera's world transform via TF (splat camera
            # publishes "world" -> image.frame_id every render tick).
            # `obs.data` here is ImageDetections2D (the .map() output);
            # the original Image is on .image.
            camera_frame = obs.data.image.frame_id
            world_to_camera = self.tf.get(
                "world", camera_frame, time_point=obs.ts, time_tolerance=2.0
            )
            if world_to_camera is None:
                # TF buffer didn't reach back this far. Fall back to
                # the pelvis pose; back-projection will likely reject
                # all points but the chain doesn't hard-fail.
                n_tf_misses += 1
                world_to_camera = Transform(
                    ts=obs.ts,
                    translation=obs.pose_stamped.position,
                    rotation=obs.pose_stamped.orientation,
                )
                logger.warning(
                    f"find_object_3d({label!r}): TF miss at ts={obs.ts:.2f} "
                    f"frame={camera_frame!r} — fell back to pelvis"
                )
            else:
                logger.info(
                    f"find_object_3d({label!r}): TF hit at ts={obs.ts:.2f} "
                    f"cam_world=({world_to_camera.translation.x:+.2f},"
                    f"{world_to_camera.translation.y:+.2f},"
                    f"{world_to_camera.translation.z:+.2f}) "
                    f"pelvis=({obs.pose_stamped.position.x:+.2f},"
                    f"{obs.pose_stamped.position.y:+.2f},"
                    f"{obs.pose_stamped.position.z:+.2f})"
                )

            for det in obs.data.detections:
                n_dets += 1
                det3d = Detection3DPC.from_2d(
                    det,
                    lidar_obs.data,
                    camera_info=cam_info,
                    world_to_optical_transform=world_to_camera.inverse(),
                )
                if det3d is None or len(det3d.pointcloud) == 0:
                    continue
                p = det3d.center
                # Publish the bbox so viser can draw it — this is the
                # only signal that lets the user distinguish a
                # perception/localization error from a pointing error.
                ext = det3d.get_bounding_box_dimensions()
                with self._found_lock:
                    self._found[label] = BBoxMarker(
                        label=label,
                        center=(float(p.x), float(p.y), float(p.z)),
                        extent=(float(ext[0]), float(ext[1]), float(ext[2])),
                    )
                    snapshot = list(self._found.values())
                self.found_objects.publish(BBoxMarkers(markers=snapshot))
                # Also publish the actual lidar subset that produced
                # this centroid — visual debug for "did the back-
                # projection select the right cluster?"
                try:
                    self.found_pointcloud.publish(det3d.pointcloud)
                except Exception as e:
                    logger.debug(f"found_pointcloud publish failed: {e}")
                logger.info(f"find_object_3d('{label}') -> ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
                return f"Found '{label}' at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"

        if n_dets == 0:
            return (
                f"Saw '{label}' in memory (CLIP hotspot at "
                f"({hotspot.pose_stamped.position.x:.2f}, "
                f"{hotspot.pose_stamped.position.y:.2f}, "
                f"{hotspot.pose_stamped.position.z:.2f})) "
                f"but VLM didn't detect '{label}' in {n_candidate_frames} "
                f"best-of-window frames around it."
            )
        # All detections projected through TF, but every back-projection
        # rejected the points. Usually means the lidar pointcloud doesn't
        # cover the 2D bbox region (target outside lidar FoV, or the
        # bbox is on a far wall the lidar can't reach).
        h = hotspot.pose_stamped.position
        tf_note = (
            f" ({n_tf_misses}/{n_candidate_frames} frames missed TF — "
            f"bump tf_buffer_size_s if your recordings are longer)"
            if n_tf_misses
            else ""
        )
        return (
            f"Found '{label}' near ({h.x:.2f}, {h.y:.2f}, {h.z:.2f}) "
            f"(VLM saw {n_dets} detections in {n_candidate_frames} frames; "
            f"3D back-projection found no lidar points in the bboxes — "
            f"target may be outside lidar FoV; reporting hotspot pose "
            f"instead{tf_note})."
        )


__all__ = ["ObjectFinder3D", "ObjectFinder3DConfig"]
