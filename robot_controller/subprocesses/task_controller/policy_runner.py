from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import onnxruntime as ort
import yaml


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp)
    if not isinstance(data, dict):
        raise ValueError(f"YAML file must contain a mapping: {path}")
    return data


def project_root(value: str | None) -> Path:
    root = Path(value) if value else Path.cwd()
    if not root.is_absolute():
        root = Path.cwd() / root
    if not root.is_dir():
        raise FileNotFoundError(f"QHRR project root not found: {root}")
    return root.resolve()


def resolve_policy_config_dir(project_root_path: Path, policy_config_dir: str, robot_name: str) -> Path:
    root = Path(policy_config_dir)
    if not root.is_absolute():
        root = project_root_path / root
    if (root / "policy_list.yaml").exists() or (root / "runner_config.yaml").exists():
        return root.resolve()
    robot_root = root / robot_name
    if not ((robot_root / "policy_list.yaml").exists() or (robot_root / "runner_config.yaml").exists()):
        raise FileNotFoundError(f"policy config directory not found: {robot_root}")
    return robot_root.resolve()


@dataclass
class RunnerBundle:
    name: str
    directory: Path
    runner_config: dict[str, Any]
    obs_config: dict[str, Any]


def load_bundles(policy_config_dir: Path) -> list[RunnerBundle]:
    policy_list_path = policy_config_dir / "policy_list.yaml"
    if policy_list_path.exists():
        policy_list = load_yaml(policy_list_path)
        names = policy_list.get("list_of_policy_names")
        if not isinstance(names, list) or not names:
            raise ValueError(f"invalid policy list: {policy_list_path}")
        bundles = []
        for name in names:
            session_dir = policy_config_dir / str(name)
            bundles.append(
                RunnerBundle(
                    name=str(name),
                    directory=session_dir,
                    runner_config=load_yaml(session_dir / "runner_config.yaml"),
                    obs_config=load_yaml(session_dir / "obs_config.yaml")
                    if (session_dir / "obs_config.yaml").exists()
                    else {},
                )
            )
        return bundles
    return [
        RunnerBundle(
            name=policy_config_dir.name,
            directory=policy_config_dir,
            runner_config=load_yaml(policy_config_dir / "runner_config.yaml"),
            obs_config=load_yaml(policy_config_dir / "obs_config.yaml")
            if (policy_config_dir / "obs_config.yaml").exists()
            else {},
        )
    ]


def load_policies(project_root_path: Path, policy_config_dir: Path) -> list[OnnxPolicy]:
    return [OnnxPolicy(project_root_path, bundle) for bundle in load_bundles(policy_config_dir)]


def action_offset(policy: OnnxPolicy, robot_name: str, q: np.ndarray, mode: bool) -> np.ndarray:
    defaults = np.asarray([float(value) for value in policy.config["default_joint_angle"]], dtype=np.float32)
    defaults = defaults[: len(q)]
    if robot_name in {"qhrr", "qhrr1"}:
        # print(defaults)
        return defaults
    # if robot_name == "rbq":
    #     quad = np.resize(np.asarray([0.0, 0.7, -1.4], dtype=np.float32), len(q))
    #     alpha = max(0.0, min(1.0, -policy.projected_gravity[2]))
    #     beta = max(0.0, min(1.0, -policy.projected_gravity[0])) * float(mode)
    #     return alpha * (quad - q) + beta * (defaults - q) + q
    return defaults


def _strip_leading_parents(path: Path) -> Path:
    parts = list(path.parts)
    while parts and parts[0] in (".", ".."):
        parts.pop(0)
    return Path(*parts) if parts else path


def _resolve_policy_dir(project_root_path: Path, raw_file_path: str) -> Path:
    raw = Path(raw_file_path)
    candidates = [raw] if raw.is_absolute() else [project_root_path / raw, project_root_path / _strip_leading_parents(raw)]
    for candidate in candidates:
        if (candidate / "policy.onnx").exists():
            return candidate.resolve()
    raise FileNotFoundError(f"policy.onnx not found from file_path: {raw_file_path}")


def _quat_to_projected_gravity(quat_wxyz: list[float]) -> np.ndarray:
    w, x, y, z = quat_wxyz
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )
    pg = rotation.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)
    return np.array([pg[0], pg[1], pg[2]])


class OnnxPolicy:
    def __init__(self, project_root_path: Path, bundle: RunnerBundle) -> None:
        self.name = bundle.name
        self.directory = bundle.directory
        self.config = dict(bundle.runner_config)
        self.obs_config = dict(bundle.obs_config)
        self.num_joint = int(self.config["num_joint"])
        self.action_scale = float(self.config["action_scale"])
        self.action_clip = float(self.config["action_clip"])
        self.obs_type = int(self.config.get("obs_type", 0))
        self.joint_idx_style = str(self.config["joint_idx_style"])
        conversion = self.config[f"joint_idx_conversion_{self.joint_idx_style}"]
        self.input_indices = [int(value) for value in conversion["input"]]
        self.output_indices = [int(value) for value in conversion["output"]]
        defaults = [float(value) for value in self.config["default_joint_angle"]]
        self.default_joint_angle = np.array([defaults[index] for index in self.input_indices], dtype=np.float32)
        self.dof_pos = np.zeros(self.num_joint, dtype=np.float32)
        self.dof_vel = np.zeros(self.num_joint, dtype=np.float32)
        self.delta_dof_pos = np.zeros(self.num_joint, dtype=np.float32)
        self.actions = np.zeros(self.num_joint, dtype=np.float32)
        self.scaled_actions = np.zeros(self.num_joint, dtype=np.float32)
        self.last_actions = np.zeros(self.num_joint, dtype=np.float32)
        self.last_last_actions = np.zeros(self.num_joint, dtype=np.float32)
        self.last_last_last_actions = np.zeros(self.num_joint, dtype=np.float32)
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self.commands = np.zeros(3, dtype=np.float32)
        self.mode = False
        self.obs_components = self._obs_components()
        self.obs_scales = self._obs_scales()
        policy_dir = _resolve_policy_dir(project_root_path, str(self.config["file_path"]))
        self.session = ort.InferenceSession(str(policy_dir / "policy.onnx"), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        print(f"[task_controller] loaded policy {self.name}: {policy_dir / 'policy.onnx'}", flush=True)
        self.cnt = 0

    def set_state(self, dof_pos: np.ndarray, dof_vel: np.ndarray, quat_wxyz: list[float], ang_vel: np.ndarray) -> None:
        self.dof_pos[:] = np.array([dof_pos[index] for index in self.input_indices], dtype=np.float32)
        self.dof_vel[:] = np.array([dof_vel[index] for index in self.input_indices], dtype=np.float32)
        if self.obs_type == 1:
            self.delta_dof_pos[:] = self.dof_pos
        else:
            self.delta_dof_pos[:] = self.dof_pos - self.default_joint_angle
        self.projected_gravity = _quat_to_projected_gravity(quat_wxyz)
        self.base_ang_vel[:] = ang_vel

    def set_commands(self, lin_x: float, lin_y: float, yaw: float, mode: bool) -> None:
        self.commands[:] = [lin_x, lin_y, yaw]
        self.mode = bool(mode)

    def compute_action(self) -> np.ndarray:
        self.cnt += 1
        # print(f"[task_controller] Compute action call {self.cnt}")
        output = self.session.run([self.output_name], {self.input_name: self._observation()})[0]
        raw = np.asarray(output, dtype=np.float32).reshape(-1)[: self.num_joint]
        self.actions[:] = np.clip(raw, -self.action_clip, self.action_clip)
        scaled_original = self.actions * self.action_scale
        for out_index, src_index in enumerate(self.output_indices):
            self.scaled_actions[out_index] = scaled_original[src_index]
        self.last_last_last_actions[:] = self.last_last_actions
        self.last_last_actions[:] = self.last_actions
        self.last_actions[:] = self.actions
        return self.scaled_actions.copy()

    def _observation(self) -> np.ndarray:
        values: list[float] = []
        alpha = max(0.0, min(1.0, -self.projected_gravity[2]))
        beta = max(0.0, min(1.0, -self.projected_gravity[0])) * float(self.mode)
        for component in self.obs_components:
            if component == "base_ang_vel_":
                values.extend((self.base_ang_vel * self._scale(component, 1.0)).tolist())
            elif component == "projected_gravity":
                values.extend((self.projected_gravity * self._scale(component, 1.0)).tolist())
                # print(f"[task_controller] Projected gravity: {self.projected_gravity}")
            elif component == "lin_vel_x_commands_":
                values.append(self.commands[0] * self._scale(component, 1.0))
            elif component == "lin_vel_y_commands_":
                values.append(self.commands[1] * self._scale(component, 1.0))
            elif component == "ang_vel_z_commands_":
                values.append(self.commands[2] * self._scale(component, 1.0))
            elif component == "dof_pos":
                values.extend((self.dof_pos * self._scale(component, 1.0)).tolist())
            elif component == "delta_dof_pos":
                values.extend((self.delta_dof_pos * self._scale(component, 1.0)).tolist())
            elif component == "dof_vel":
                values.extend((self.dof_vel * self._scale(component, 1.0)).tolist())
            elif component == "actions":
                values.extend((self.actions * self._scale(component, 1.0)).tolist())
            elif component == "last_actions":
                values.extend((self.last_actions * self._scale(component, 1.0)).tolist())
            elif component == "last_last_actions":
                values.extend((self.last_last_actions * self._scale(component, 1.0)).tolist())
            elif component == "last_last_last_actions":
                values.extend((self.last_last_last_actions * self._scale(component, 1.0)).tolist())
            elif component == "alpha":
                values.append(alpha * self._scale(component, 1.0))
            elif component == "beta":
                values.append(beta * self._scale(component, 1.0))
            elif component == "mode":
                values.append(float(self.mode) * self._scale(component, 1.0))
            else:
                raise RuntimeError(f"unsupported observation component: {component}")
        return np.asarray(values, dtype=np.float32)[None, :]

    def _obs_components(self) -> list[str]:
        observations = self.obs_config.get("observations", {})
        components = observations.get("components", [])
        if components:
            return [str(value) for value in components]
        else: raise RuntimeError(f"No observation components specified")

    def _obs_scales(self) -> dict[str, float]:
        observations = self.obs_config.get("observations", {})
        scales = observations.get("scales", {})
        if not isinstance(scales, dict):
            return {}
        return {str(key): float(value) for key, value in scales.items()}

    def _scale(self, key: str, default: float) -> float:
        return self.obs_scales.get(key, default)
