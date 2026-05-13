from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from sequence_module import DumeSequenceModule, SequenceSession, SequenceStep


@dataclass(slots=True)
class LiveRecorderSnapshot:
    running: bool
    session_name: str | None
    dume_base_url: str | None
    steps_recorded: int
    last_error: str | None
    included_continuous_joint: str | None
    excluded_continuous_joints: list[str]
    controller_connected: bool | None


class LiveSequenceRecorder:
    def __init__(self, root: Path, config: dict, poll_interval_s: float = 0.12) -> None:
        self.root = root
        self.config = config
        self.poll_interval_s = poll_interval_s
        self.module = DumeSequenceModule(root=root, config=config)

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self._session: SequenceSession | None = None
        self._last_error: str | None = None
        self._previous_joints: dict[str, dict[str, Any]] | None = None
        self._previous_inputs: dict[str, Any] | None = None
        self._active_continuous: dict[str, Any] | None = None
        self._controller_connected: bool | None = None

    def start(self, session_name: str, *, overwrite_existing: bool = False) -> SequenceSession:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("Recorder is already running.")
            session_path = self.module.session_path(session_name)
            if overwrite_existing and session_path.exists():
                session_path.unlink()
            self._stop_event.clear()
            self._last_error = None
            self._previous_joints = None
            self._previous_inputs = None
            self._active_continuous = None
            self._controller_connected = None
            self._session = self.module.create_session(session_name)
            self._thread = threading.Thread(target=self._run_loop, name="dume-sequence-recorder", daemon=True)
            self._thread.start()
            return self._session

    def stop(self) -> SequenceSession | None:
        with self._lock:
            thread = self._thread
            self._stop_event.set()
        if thread is not None:
            thread.join(timeout=5.0)
        with self._lock:
            self._thread = None
            if self._session is not None:
                self.module.save_session(self._session)
            return self._session

    def snapshot(self) -> LiveRecorderSnapshot:
        with self._lock:
            running = self._thread is not None and self._thread.is_alive()
            return LiveRecorderSnapshot(
                running=running,
                session_name=self._session.session_name if self._session is not None else None,
                dume_base_url=self._session.dume_base_url if self._session is not None else None,
                steps_recorded=len(self._session.steps) if self._session is not None else 0,
                last_error=self._last_error,
                included_continuous_joint=self._session.included_continuous_joint if self._session is not None else None,
                excluded_continuous_joints=list(self._session.excluded_continuous_joints) if self._session is not None else [],
                controller_connected=self._controller_connected,
            )

    def _run_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                with self._lock:
                    session = self._session
                if session is None:
                    return

                payload = self.module._fetch_device_state(session.dume_base_url)
                controller_payload = payload.get("ps4", {})
                controller_inputs = controller_payload.get("inputs", {}) if isinstance(controller_payload, dict) else {}
                self._controller_connected = bool(controller_payload.get("connected"))
                current_joints = {
                    str(joint.get("name", "")): {
                        "motor_type": str(joint.get("motor_type", "")),
                        "position": int(joint.get("position", 0)),
                        "step": int(joint.get("step", 1)),
                        "control_mode": str(joint.get("control_mode", "none")),
                        "axis_source": str(joint.get("axis_source", "none")),
                        "positive_button": str(joint.get("positive_button", "none")),
                        "negative_button": str(joint.get("negative_button", "none")),
                        "input_invert": bool(joint.get("input_invert", False)),
                    }
                    for joint in payload.get("joints", [])
                    if joint.get("name")
                }

                now = time.monotonic()
                with self._lock:
                    self._ingest_joint_snapshot(now, current_joints, controller_inputs if isinstance(controller_inputs, dict) else {})
                    self.module.save_session(self._session)
                time.sleep(self.poll_interval_s)
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
        finally:
            with self._lock:
                self._finalize_active_continuous(time.monotonic())
                if self._session is not None:
                    self.module.save_session(self._session)

    def _ingest_joint_snapshot(
        self,
        timestamp_s: float,
        current_joints: dict[str, dict[str, Any]],
        controller_inputs: dict[str, Any],
    ) -> None:
        if self._session is None:
            return

        previous_joints = self._previous_joints
        self._previous_joints = current_joints
        self._previous_inputs = controller_inputs
        if previous_joints is None:
            return

        included_cont = self._session.included_continuous_joint
        for joint_name, current in current_joints.items():
            previous = previous_joints.get(joint_name, current)
            motor_type = current["motor_type"]
            current_position = int(current["position"])
            previous_position = int(previous["position"])

            if motor_type == "continuous_360":
                if joint_name != included_cont:
                    continue
                self._ingest_continuous_transition(joint_name, current, controller_inputs, current_position, timestamp_s)
                continue

            if current_position != previous_position:
                self._session.steps.append(
                    SequenceStep(
                        kind="positional_move",
                        joint_name=joint_name,
                        target_value=current_position,
                        note="Recorded from live controller-driven joint state.",
                    )
                )

    def _ingest_continuous_transition(
        self,
        joint_name: str,
        joint_state: dict[str, Any],
        current_inputs: dict[str, Any],
        current_value: int,
        timestamp_s: float,
    ) -> None:
        telemetry_value = self._continuous_command_from_inputs(joint_state, current_inputs)
        if telemetry_value is None:
            telemetry_value = current_value
        active = self._active_continuous
        if active is None:
            if telemetry_value != 0:
                self._active_continuous = {
                    "joint_name": joint_name,
                    "speed_percent": abs(int(telemetry_value)),
                    "direction": 1 if int(telemetry_value) > 0 else -1,
                    "started_at_s": timestamp_s,
                    "source": "controller_inputs",
                }
            return

        if telemetry_value == 0:
            self._finalize_active_continuous(timestamp_s)
            return

        current_direction = 1 if int(telemetry_value) > 0 else -1
        current_speed = abs(int(telemetry_value))
        if current_direction != int(active["direction"]) or current_speed != int(active["speed_percent"]):
            self._finalize_active_continuous(timestamp_s)
            self._active_continuous = {
                "joint_name": joint_name,
                "speed_percent": current_speed,
                "direction": current_direction,
                "started_at_s": timestamp_s,
                "source": "controller_inputs",
            }

    def _finalize_active_continuous(self, timestamp_s: float) -> None:
        if self._session is None or self._active_continuous is None:
            return
        active = self._active_continuous
        duration_ms = max(1, int(round((timestamp_s - float(active["started_at_s"])) * 1000.0)))
        self._session.steps.append(
            SequenceStep(
                kind="continuous_burst",
                joint_name=str(active["joint_name"]),
                speed_percent=int(active["speed_percent"]),
                direction=int(active["direction"]),
                duration_ms=duration_ms,
                note=f"Recorded from live controller-driven continuous joint state via {active.get('source', 'controller_inputs')}.",
            )
        )
        self._active_continuous = None

    @staticmethod
    def _button_state(inputs: dict[str, Any], button_name: str) -> bool:
        buttons = inputs.get("buttons", {})
        if not isinstance(buttons, dict):
            return False
        return bool(buttons.get(button_name, False))

    def _continuous_command_from_inputs(self, joint_state: dict[str, Any], inputs: dict[str, Any]) -> int | None:
        control_mode = str(joint_state.get("control_mode", "none"))
        input_invert = bool(joint_state.get("input_invert", False))
        step = max(1, int(joint_state.get("step", 1)))

        if control_mode == "buttons":
            positive_name = str(joint_state.get("positive_button", "none"))
            negative_name = str(joint_state.get("negative_button", "none"))
            direction = 0
            if positive_name != "none" and self._button_state(inputs, positive_name):
                direction += 1
            if negative_name != "none" and self._button_state(inputs, negative_name):
                direction -= 1
            if input_invert:
                direction *= -1
            if direction == 0:
                return 0
            return direction * step * 10

        if control_mode == "axis":
            axis_source = str(joint_state.get("axis_source", "none"))
            axis_key_map = {
                "left_stick_x": "centered_lx",
                "left_stick_y": "centered_ly",
                "right_stick_x": "centered_rx",
                "right_stick_y": "centered_ry",
                "dpad_x": "dpad_x",
                "dpad_y": "dpad_y",
                "triggers": "trigger_difference",
            }
            axis_key = axis_key_map.get(axis_source)
            if axis_key is None:
                return 0
            raw_value = int(inputs.get(axis_key, 0))
            if input_invert:
                raw_value *= -1
            deadzone = int(inputs.get("axis_deadzone", self.config.get("controller", {}).get("axis_deadzone", 48)))
            if abs(raw_value) < deadzone:
                return 0
            max_magnitude = 1023 if axis_source == "triggers" else 512
            scaled = int(round((max(-max_magnitude, min(max_magnitude, raw_value)) / float(max_magnitude)) * 100.0))
            return max(-100, min(100, scaled))

        return None
