from __future__ import annotations

import json
import time
from typing import Any

from robot_controller.shm.robot_state import RobotStateShm as RobotStateShmReader

from .state import MonitorState, hex_id


class DashboardRobotStateReader:
    def __init__(
        self,
        control_shm_name: str,
        dashboard_shm_name: str,
        stale_timeout_s: float,
        state: MonitorState,
    ) -> None:
        self.control_shm_name = control_shm_name
        self.dashboard_shm_name = dashboard_shm_name
        self.stale_timeout_s = float(stale_timeout_s)
        self.state = state
        self.control_reader = RobotStateShmReader(control_shm_name)
        self.dashboard_reader = RobotStateShmReader(dashboard_shm_name)

    def close(self) -> None:
        self.control_reader.close()
        self.dashboard_reader.close()

    def dashboard_snapshot(self) -> dict[str, Any]:
        base = self.state.snapshot()
        control_state = self._read_channel(
            reader=self.control_reader,
            shm_key="control_state",
            shm_name=self.control_shm_name,
            expected_schema="qhrr.control_state.v1",
        )
        base["shm"]["control_state"] = control_state["shm"]
        base["robot_controller"] = self._controller_snapshot(control_state)

        if control_state["payload"] is not None:
            self._merge_control_state(base, control_state["payload"])

        dashboard_state = self._read_channel(
            reader=self.dashboard_reader,
            shm_key="dashboard_state",
            shm_name=self.dashboard_shm_name,
            expected_schema="qhrr.dashboard_state.v1",
        )
        base["shm"]["dashboard_state"] = dashboard_state["shm"]

        if dashboard_state["payload"] is not None:
            self._merge_dashboard_state(base, dashboard_state["payload"])

        return base

    def _read_channel(
        self,
        *,
        reader: RobotStateShmReader,
        shm_key: str,
        shm_name: str,
        expected_schema: str,
    ) -> dict[str, Any]:
        try:
            robot_state = reader.read_latest()
        except FileNotFoundError as exc:
            return self._channel_result(shm_key, shm_name, "disconnected", None, str(exc))
        except (RuntimeError, ValueError, json.JSONDecodeError) as exc:
            return self._channel_result(shm_key, shm_name, "error", None, str(exc))

        if robot_state is None:
            return self._channel_result(shm_key, shm_name, "waiting", None, None)

        schema = str(robot_state.get("schema", ""))
        if schema not in (expected_schema, "qhrr.robot_state.cstruct.v1"):
            return self._channel_result(
                shm_key,
                shm_name,
                "error",
                None,
                f"{shm_name} schema must be {expected_schema}",
            )

        try:
            timestamp_unix = float(robot_state["timestamp_unix"])
        except (KeyError, TypeError, ValueError) as exc:
            return self._channel_result(
                shm_key,
                shm_name,
                "error",
                None,
                f"{shm_name} is missing numeric timestamp_unix: {exc}",
            )

        age_s = max(0.0, time.time() - timestamp_unix)
        status = "stale" if age_s > self.stale_timeout_s else "online"
        return self._channel_result(shm_key, shm_name, status, robot_state, None, age_s=age_s)

    def _merge_control_state(self, base: dict[str, Any], robot_state: dict[str, Any]) -> None:
        now_monotonic = time.monotonic()
        imu = robot_state.get("imu", {})
        base["imu"] = {
            "req_count": base["imu"]["req_count"],
            "quat_count": base["imu"]["quat_count"],
            "gyro_count": base["imu"]["gyro_count"],
            "quat_xyzw": imu.get("quat_xyzw") or [0.0, 0.0, 0.0, 1.0],
            "projected_gravity_b": imu.get("projected_gravity_b") or [0.0, 0.0, -1.0],
            "angular_velocity_rad_s": imu.get("angular_velocity_rad_s") or [0.0, 0.0, 0.0],
            "quat_age_s": self._age(now_monotonic, self._float_or_zero(imu.get("last_quat_t"))),
            "gyro_age_s": self._age(now_monotonic, self._float_or_zero(imu.get("last_gyro_t"))),
        }

        motors = [
            self._control_motor_snapshot(item, now_monotonic)
            for item in robot_state.get("actuators", [])
            if isinstance(item, dict)
        ]
        base["motors"] = motors
        base["nodes"] = self._nodes_from_control_state(base["nodes"], motors)
        command_output = robot_state.get("command_output")
        if isinstance(command_output, dict):
            base["current_command"] = command_output

    def _merge_dashboard_state(self, base: dict[str, Any], robot_state: dict[str, Any]) -> None:
        base["processes"] = self._processes_snapshot(robot_state.get("processes", {}))
        can = robot_state.get("can", {})
        if isinstance(can, dict) and can.get("iface"):
            base["can"]["iface"] = can["iface"]

    def _control_motor_snapshot(self, item: dict[str, Any], now_monotonic: float) -> dict[str, Any]:
        can_id = int(item["can_id"])
        age_s = item.get("age_s")
        if age_s is None:
            age_s = self._age(now_monotonic, self._float_or_zero(item.get("last_feedback_t")))
        status = self._control_status(item, age_s)
        existing = self.state.motors.get(can_id)
        velocity_rad_s = item.get("velocity_rad_s")
        current_a = item.get("current_a")
        if current_a is None and existing is not None:
            current_a = existing.iq_a_approx
        temperature_c = item.get("temperature_c")
        if temperature_c is None and existing is not None:
            temperature_c = existing.temperature_c
        return {
            "name": self._actuator_name(can_id),
            "can_id": hex_id(can_id),
            "rx_count": 0 if existing is None else int(existing.rx_count),
            "age_s": age_s,
            "status": status,
            "last_kind": self._control_mode(item, age_s),
            "enabled_hint": item.get("is_enabled"),
            "mit_polling": can_id in self.state.mit_poll_can_ids,
            "temperature_c": temperature_c,
            "current_a": current_a,
            "iq_a_approx": current_a,
            "speed_dps": self._rad_s_to_deg_s(velocity_rad_s),
            "position_rad": item.get("position_rad"),
            "zero_offset_count": None if existing is None else existing.zero_offset_count,
            "raw": self._control_summary(item),
        }

    def _nodes_from_control_state(
        self,
        existing_nodes: list[dict[str, Any]],
        motors: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        non_actuator_nodes = [
            node for node in existing_nodes
            if node.get("role") != "Actuator"
        ]
        existing_by_can_id = {
            int(str(node["can_id"]), 0): node
            for node in existing_nodes
            if "can_id" in node
        }
        actuator_nodes = []
        for motor in motors:
            can_id = int(str(motor["can_id"]), 0)
            existing = existing_by_can_id.get(can_id, {})
            actuator_nodes.append(
                {
                    "key": f"can_{can_id:03X}",
                    "name": motor["name"],
                    "can_id": motor["can_id"],
                    "role": "Actuator",
                    "heartbeat_hz": existing.get("heartbeat_hz", 0.0),
                    "last_seen_s": motor["age_s"],
                    "timeout_s": self.state.node_timeout_s,
                    "status": motor["status"],
                    "rx_count": int(motor["rx_count"]),
                    "last_data": motor["raw"],
                }
            )
        return sorted(non_actuator_nodes + actuator_nodes, key=lambda item: int(str(item["can_id"]), 0))

    def _channel_result(
        self,
        shm_key: str,
        shm_name: str,
        status: str,
        payload: dict[str, Any] | None,
        error: str | None,
        *,
        age_s: float | None = None,
    ) -> dict[str, Any]:
        return {
            "key": shm_key,
            "payload": payload,
            "shm": {
                "status": status,
                "shm_name": shm_name,
                "age_s": age_s,
                "error": error,
                "payload": payload,
            },
        }

    @staticmethod
    def _controller_snapshot(control_state: dict[str, Any]) -> dict[str, Any]:
        shm = control_state["shm"]
        payload = control_state["payload"]
        controller_state = None
        if isinstance(payload, dict):
            controller_state = payload.get("controller_state")
            safety_state = payload.get("safety_state") or controller_state
            control_action = payload.get("control_action")
            safety_reason = payload.get("safety_reason")
            fault_code = payload.get("fault_code")
        else:
            safety_state = None
            control_action = None
            safety_reason = None
            fault_code = None
        return {
            "source": "control_shm",
            "shm_name": shm["shm_name"],
            "status": shm["status"],
            "controller_state": controller_state,
            "safety_state": safety_state,
            "control_action": control_action,
            "safety_reason": safety_reason,
            "fault_code": fault_code,
            "age_s": shm["age_s"],
            "error": shm["error"],
        }

    @staticmethod
    def _processes_snapshot(processes: Any) -> list[dict[str, Any]]:
        if not isinstance(processes, dict):
            return []
        rows = []
        for name, info in sorted(processes.items()):
            if not isinstance(info, dict):
                continue
            row = dict(info)
            row["name"] = str(name)
            rows.append(row)
        return rows

    def _actuator_name(self, can_id: int) -> str:
        config = self.state.actuator_config_for_can_id(can_id)
        if config is None:
            return hex_id(can_id)
        return str(config["name"])

    @staticmethod
    def _control_status(item: dict[str, Any], age_s: float | None) -> str:
        if age_s is None:
            return "never"
        if bool(item.get("stale")) or not bool(item.get("online", True)):
            return "timeout"
        return "online"

    @staticmethod
    def _control_mode(item: dict[str, Any], age_s: float | None) -> str:
        if age_s is None:
            return "never"
        if item.get("is_enabled") is True:
            return "ENABLED"
        if item.get("is_enabled") is False:
            return "DISABLED"
        return "FEEDBACK"

    @staticmethod
    def _control_summary(item: dict[str, Any]) -> str:
        parts = []
        if item.get("position_rad") is not None:
            parts.append(f"q={float(item['position_rad']):.3f}rad")
        if item.get("velocity_rad_s") is not None:
            parts.append(f"qd={float(item['velocity_rad_s']):.3f}rad/s")
        if item.get("torque_nm") is not None:
            parts.append(f"tau={float(item['torque_nm']):.3f}Nm")
        if item.get("current_a") is not None:
            parts.append(f"i={float(item['current_a']):.2f}A")
        fault_code = item.get("fault_code")
        if fault_code not in (None, 0):
            parts.append(f"fault={fault_code}")
        return " ".join(parts)

    @staticmethod
    def _rad_s_to_deg_s(value: Any) -> float | None:
        if value is None:
            return None
        return float(value) * 57.29577951308232

    @staticmethod
    def _age(now_monotonic: float, last_t: float) -> float | None:
        if last_t <= 0.0:
            return None
        return max(0.0, now_monotonic - last_t)

    @staticmethod
    def _float_or_zero(value: Any) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0
