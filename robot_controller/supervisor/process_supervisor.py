from __future__ import annotations

import logging
import os
import signal
import shlex
import subprocess
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from ..core.config import ProcessConfig


logger = logging.getLogger(__name__)


class _ProcessSupervisorBase:
    def __init__(self, process_configs: list[ProcessConfig]):
        self.process_configs = {
            config.name: config
            for config in process_configs
        }
        self.processes: dict[str, subprocess.Popen] = {}
        self.pid_dir = Path("/tmp/qhrr_robot_controller_processes")
        self.log_dir = Path.cwd() / "log" / datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._log_handles: dict[str, object] = {}
        self._pidfiles: dict[str, Path] = {
            name: self.pid_dir / f"{name}.pid"
            for name in self.process_configs
        }

    def start_by_name(self, name: str) -> None:
        if not name:
            raise ValueError("Process name must not be empty")
        config = self.process_configs.get(name)
        if config is None:
            raise KeyError(f"Unknown process: {name}")
        if self.is_alive(name):
            return
        if not config.command:
            raise ValueError(f"Process {name} has empty command")
        pidfile = self._pidfiles[name]
        pidfile.parent.mkdir(parents=True, exist_ok=True)
        if pidfile.exists():
            pidfile.unlink()
        log_file = self._log_file_for(name)
        stdout = None
        stderr = None
        if not config.new_terminal:
            handle = log_file.open("ab", buffering=0)
            self._log_handles[name] = handle
            stdout = handle
            stderr = subprocess.STDOUT
        self.processes[name] = subprocess.Popen(
            self._launch_command(config, pidfile, log_file),
            cwd=config.working_dir,
            env=self._process_env(config),
            stdout=stdout,
            stderr=stderr,
            start_new_session=True,
        )

    def start_all(self) -> None:
        configs = sorted(
            self.process_configs.values(),
            key=lambda item: item.start_order,
        )
        for config in configs:
            self.start_by_name(config.name)

    def stop_by_name(self, name: str, timeout_s: float = 2.0) -> None:
        process = self.processes.get(name)
        pidfile = self._pidfiles.get(name)
        managed_pid = self._read_pidfile(pidfile)
        if process is None and managed_pid is None:
            return

        if managed_pid is not None:
            self._terminate_pid(managed_pid, signal.SIGTERM)
        if process is not None and process.poll() is None:
            self._terminate_process_group(process, signal.SIGTERM)

        deadline = time.monotonic() + timeout_s
        if managed_pid is not None:
            self._wait_pid_exit(managed_pid, deadline)
        if process is None:
            if managed_pid is not None and self._pid_is_running(managed_pid):
                logger.warning("Killing managed process %s after stop timeout %.3fs", name, timeout_s)
                self._terminate_pid(managed_pid, signal.SIGKILL)
                self._wait_pid_exit(managed_pid, time.monotonic() + timeout_s)
            self._cleanup_pidfile(pidfile)
            return

        try:
            remaining_s = max(0.0, deadline - time.monotonic())
            process.wait(timeout=remaining_s)
        except subprocess.TimeoutExpired:
            logger.warning("Killing process %s after stop timeout %.3fs", name, timeout_s)
            if managed_pid is not None:
                self._terminate_pid(managed_pid, signal.SIGKILL)
            self._terminate_process_group(process, signal.SIGKILL)
            process.wait(timeout=timeout_s)
        finally:
            self._cleanup_pidfile(pidfile)
            self._close_log_handle(name)

    def stop_all(self, timeout_s: float = 2.0) -> None:
        names = sorted(
            self.process_configs,
            key=lambda name: self.process_configs[name].stop_order,
        )
        for name in names:
            self.stop_by_name(name, timeout_s=timeout_s)

    def is_alive(self, name: str) -> bool:
        process = self.processes.get(name)
        if process is not None and process.poll() is None:
            return True
        managed_pid = self._read_pidfile(self._pidfiles.get(name))
        return managed_pid is not None and self._pid_is_running(managed_pid)

    @staticmethod
    def _launch_command(
        config: ProcessConfig,
        pidfile: Path | None = None,
        log_file: Path | None = None,
    ) -> list[str]:
        if not config.new_terminal:
            return list(config.command)
        if not config.terminal_command:
            raise ValueError(f"Process {config.name} requires terminal_command")

        workdir = Path(config.working_dir).resolve()
        pidfile_part = ""
        if pidfile is not None:
            pidfile_part = f"printf '%s\\n' \"$$\" > {shlex.quote(str(pidfile))} && "
        log_part = ""
        if log_file is not None:
            log_part = (
                f"exec > >(tee -a {shlex.quote(str(log_file))}) 2>&1 && "
                f"printf '[process_supervisor] log file: %s\\n' {shlex.quote(str(log_file))} && "
            )
        shell_command = (
            f"cd {shlex.quote(str(workdir))} && "
            f"{pidfile_part}"
            f"{log_part}"
            f"exec {shlex.join(config.command)}"
        )
        return list(config.terminal_command) + ["bash", "-lc", shell_command]

    @staticmethod
    def _terminate_process_group(process: subprocess.Popen, signum: int) -> None:
        try:
            os.killpg(os.getpgid(process.pid), signum)
        except ProcessLookupError:
            return
        except PermissionError as exc:
            logger.warning("Failed to signal process group for pid %s: %s", process.pid, exc)
            if signum == signal.SIGTERM:
                process.terminate()
            else:
                process.kill()

    @staticmethod
    def _terminate_pid(pid: int, signum: int) -> None:
        try:
            os.kill(pid, signum)
        except ProcessLookupError:
            return
        except PermissionError as exc:
            logger.warning("Failed to signal managed pid %s: %s", pid, exc)

    @staticmethod
    def _read_pidfile(pidfile: Path | None) -> int | None:
        if pidfile is None or not pidfile.exists():
            return None
        try:
            value = int(pidfile.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            return None
        return value if value > 1 else None

    @staticmethod
    def _wait_pid_exit(pid: int, deadline: float) -> None:
        while time.monotonic() < deadline:
            if not _ProcessSupervisorBase._pid_is_running(pid):
                return
            time.sleep(0.02)

    @staticmethod
    def _pid_is_running(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    @staticmethod
    def _cleanup_pidfile(pidfile: Path | None) -> None:
        if pidfile is None:
            return
        try:
            pidfile.unlink()
        except FileNotFoundError:
            return

    def _log_file_for(self, name: str) -> Path:
        return self.log_dir / f"{name}.log"

    def _close_log_handle(self, name: str) -> None:
        handle = self._log_handles.pop(name, None)
        if handle is not None:
            handle.close()

    @staticmethod
    def _process_env(config: ProcessConfig) -> dict[str, str]:
        env = dict(os.environ)
        env.update(config.env)
        return env

    def status(self) -> dict[str, object]:
        return {
            name: {
                "config": asdict(config),
                "pid": self.processes[name].pid if name in self.processes else None,
                "managed_pid": self._read_pidfile(self._pidfiles.get(name)),
                "alive": self.is_alive(name),
                "returncode": self.processes[name].poll() if name in self.processes else None,
                "log_file": str(self._log_file_for(name)),
            }
            for name, config in self.process_configs.items()
        }


@dataclass(frozen=True)
class ProcessHealth:
    can_daemon_alive: bool
    task_controller_alive: bool
    aux_reader_alive: bool
    dashboard_alive: bool


class ProcessSupervisor(_ProcessSupervisorBase):
    def health(self) -> ProcessHealth:
        return ProcessHealth(
            can_daemon_alive=self.is_alive("can_daemon"),
            task_controller_alive=self.is_alive("task_controller"),
            aux_reader_alive=self.is_alive("aux_reader"),
            dashboard_alive=self.is_alive("dashboard"),
        )
