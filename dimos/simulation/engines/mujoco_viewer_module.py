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

"""Optional MuJoCo passive-viewer Module.

MuJoCo's ``viewer.launch_passive`` needs the main thread on macOS
(glfw constraint), which means it can't run inside a dimos worker.
This Module owns the subprocess lifecycle: on ``start()`` it spawns a
view-only entry point (``dimos.simulation.engines.mujoco_engine
view_main``) that mirrors the live ``/coordinator/joint_state`` +
``/odom`` LCM streams into a render-only ``MjData`` window; on
``stop()`` it terminates the subprocess cleanly.

Previously this code lived module-level in the GR00T sim blueprint
where the subprocess was launched as a side effect of import — that
violated the "blueprints are declarations, no logic at import time"
rule both reviewers flagged. Moving it into a real Module gives the
process a proper lifecycle (start/stop, atexit fallback) and lets
the user enable it via the standard CLI/config path
(``-o mujocoviewermodule.enabled=true``) instead of an env var.

Spawned only from MainProcess — workers are daemonic and can't
spawn children.
"""

from __future__ import annotations

import atexit
import multiprocessing as _mp
import shutil
import subprocess
import sys
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MujocoViewerModuleConfig(ModuleConfig):
    """Configuration for the optional MuJoCo passive viewer."""

    # MJCF path to load. Must match the path the in-process engine
    # compiled so the viewer's joint name → qpos index map lines up.
    mjcf_path: str = ""
    # Set False to register the module without actually spawning a
    # viewer — handy for blueprints that always declare the module
    # and let operators flip it on via CLI.
    enabled: bool = False
    # Subprocess kill grace before SIGKILL.
    terminate_timeout_s: float = 3.0


class MujocoViewerModule(Module):
    """Spawn a passive MuJoCo viewer subprocess for the duration of the
    Module's lifecycle. No physics, just rendering — mirrors what the
    in-process engine publishes."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._proc: subprocess.Popen | None = None
        self._atexit_registered = False

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.enabled:
            logger.info("MujocoViewerModule disabled (config.enabled=False); skipping")
            return
        if _mp.current_process().name != "MainProcess":
            # Worker imports of this module must be no-ops — workers
            # are daemonic and can't spawn children.
            logger.debug("MujocoViewerModule: not MainProcess, skipping spawn")
            return
        if not self.config.mjcf_path:
            logger.warning("MujocoViewerModule enabled but mjcf_path is empty; skipping")
            return

        # mujoco.viewer.launch_passive needs ``mjpython`` on macOS.
        if sys.platform == "darwin":
            viewer_python = shutil.which("mjpython") or shutil.which("python")
        else:
            viewer_python = sys.executable
        if viewer_python is None:
            logger.warning(
                "MujocoViewerModule enabled but no mjpython/python on PATH; not launched"
            )
            return

        self._proc = subprocess.Popen(
            [viewer_python, "-m", "dimos.simulation.engines.mujoco_engine", self.config.mjcf_path],
        )
        logger.info(
            f"MujocoViewerModule spawned (pid={self._proc.pid}, "
            f"executable={viewer_python}, mjcf={self.config.mjcf_path})"
        )

        # Belt-and-suspenders: if the host process dies without calling
        # stop() (uncaught exception, SIGKILL etc.), atexit best-effort
        # cleans up the viewer subprocess.
        if not self._atexit_registered:
            atexit.register(self._terminate)
            self._atexit_registered = True

    @rpc
    def stop(self) -> None:
        self._terminate()
        super().stop()

    def _terminate(self) -> None:
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=self.config.terminate_timeout_s)
            logger.info(f"MujocoViewerModule subprocess pid={proc.pid} terminated")
        except subprocess.TimeoutExpired:
            logger.warning(
                f"MujocoViewerModule subprocess pid={proc.pid} didn't terminate in "
                f"{self.config.terminate_timeout_s:.1f}s; SIGKILL"
            )
            proc.kill()
        except Exception as e:
            logger.warning(f"MujocoViewerModule termination raised: {e}")
        finally:
            self._proc = None


__all__ = ["MujocoViewerModule", "MujocoViewerModuleConfig"]
