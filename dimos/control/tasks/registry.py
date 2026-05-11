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

"""ControlTask registry with auto-discovery.

Each task module exposes a ``register(registry)`` function that the
registry calls during discovery — same pattern as
``WholeBodyAdapterRegistry``. The registry replaces a long
``if/elif task_type == "..."`` chain in ``ControlCoordinator`` with a
single dispatch table.

Factories receive ``(cfg: TaskConfig, *, hardware: Mapping[str,
ConnectedHardware | ConnectedWholeBody])`` so tasks that need hardware
(e.g. ``G1GrootWBCTask`` pulls the WholeBodyAdapter for IMU + 29-DOF
state) can resolve their dependency through the same hook everyone
else uses.

Usage:
    from dimos.control.tasks.registry import control_task_registry

    task = control_task_registry.create(cfg.type, cfg, hardware=self._hardware)
    print(control_task_registry.available())
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
import importlib
import os
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody
    from dimos.control.task import ControlTask

logger = setup_logger()


# A task factory takes the TaskConfig and a hardware mapping and returns
# a ready-to-add ControlTask. Most factories ignore the hardware arg.
TaskFactory = Callable[..., "ControlTask"]


class ControlTaskRegistry:
    """Registry for control-task factories with auto-discovery."""

    def __init__(self) -> None:
        self._factories: dict[str, TaskFactory] = {}

    def register(self, name: str, factory: TaskFactory) -> None:
        """Register a task factory under ``name`` (case-insensitive)."""
        self._factories[name.lower()] = factory

    def create(
        self,
        name: str,
        cfg: TaskConfig,
        *,
        hardware: Mapping[str, ConnectedHardware | ConnectedWholeBody] | None = None,
    ) -> ControlTask:
        """Instantiate a task by registered name.

        Args:
            name: Registered task-type name (e.g. ``"trajectory"``).
            cfg: ``TaskConfig`` carrying name/joint_names/priority and
                whatever else this task needs.
            hardware: Coordinator's hardware map. Tasks that need an
                adapter resolve via ``cfg.hardware_id``; pass ``None``
                only if no task in this registry needs hardware.
        """
        key = name.lower()
        if key not in self._factories:
            raise ValueError(
                f"Unknown task type: {name!r}. Available: {self.available()}"
            )
        return self._factories[key](cfg=cfg, hardware=hardware or {})

    def available(self) -> list[str]:
        return sorted(self._factories.keys())

    def discover(self) -> None:
        """Import every ``dimos/control/tasks/*.py`` module and call its
        ``register(registry)`` hook if it has one. Modules without that
        hook (helpers, the registry itself, the legacy ``task.py`` base
        class) are silently skipped."""
        import dimos.control.tasks as pkg

        pkg_dir = pkg.__path__[0]
        for entry in sorted(os.listdir(pkg_dir)):
            if not entry.endswith(".py") or entry.startswith("_"):
                continue
            if entry in {"registry.py"}:
                continue
            mod_name = f"dimos.control.tasks.{entry[:-3]}"
            try:
                mod = importlib.import_module(mod_name)
            except ImportError as e:
                logger.warning(f"Skipping task module {mod_name}: {e}")
                continue
            register = getattr(mod, "register", None)
            if callable(register):
                register(self)


control_task_registry = ControlTaskRegistry()
control_task_registry.discover()

__all__ = ["ControlTaskRegistry", "TaskFactory", "control_task_registry"]
