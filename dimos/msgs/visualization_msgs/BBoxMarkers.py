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

"""3D bounding-box markers for live overlay in viser/rerun.

A snapshot replaces the previous overlay completely — boxes missing
from a snapshot disappear from the scene, so the visualizer always
matches the latest publication.
"""

from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
import json
import struct
import time

from dimos.types.timestamped import Timestamped


@dataclass
class BBoxMarker:
    """Axis-aligned bounding box in world frame.

    Attributes:
        label: human-readable name (e.g. "chair").
        center: (x, y, z) in world frame, meters.
        extent: (sx, sy, sz) box dimensions, meters.
    """

    label: str
    center: tuple[float, float, float]
    extent: tuple[float, float, float]


class BBoxMarkers(Timestamped):
    """Snapshot of labelled 3D bboxes. Wire format: JSON over LCM."""

    msg_name = "visualization_msgs.BBoxMarkers"

    def __init__(
        self,
        markers: list[BBoxMarker] | None = None,
        ts: float | None = None,
    ) -> None:
        self.markers: list[BBoxMarker] = markers or []
        self.ts: float = ts or time.time()

    def _encode_one(self, buf: BytesIO) -> None:
        payload = json.dumps(
            [
                {
                    "label": m.label,
                    "center": list(m.center),
                    "extent": list(m.extent),
                }
                for m in self.markers
            ]
        ).encode()
        buf.write(struct.pack(">d", self.ts))
        buf.write(struct.pack(">I", len(payload)))
        buf.write(payload)

    def encode(self) -> bytes:
        buf = BytesIO()
        self._encode_one(buf)
        return buf.getvalue()

    def lcm_encode(self) -> bytes:
        return self.encode()

    @classmethod
    def _decode_one(cls, buf: BytesIO) -> BBoxMarkers:
        (ts,) = struct.unpack(">d", buf.read(8))
        (length,) = struct.unpack(">I", buf.read(4))
        payload = json.loads(buf.read(length).decode())
        markers = [
            BBoxMarker(
                label=m["label"],
                center=tuple(m["center"]),  # type: ignore[arg-type]
                extent=tuple(m["extent"]),  # type: ignore[arg-type]
            )
            for m in payload
        ]
        return cls(markers=markers, ts=ts)

    @classmethod
    def decode(cls, data: bytes) -> BBoxMarkers:
        return cls._decode_one(BytesIO(data))

    @classmethod
    def lcm_decode(cls, data: bytes) -> BBoxMarkers:
        return cls.decode(data)


__all__ = ["BBoxMarker", "BBoxMarkers"]
