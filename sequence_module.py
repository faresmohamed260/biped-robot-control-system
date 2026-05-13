from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import requests


@dataclass(slots=True)
class SequenceStep:
    kind: str
    joint_name: str | None = None
    target_value: int | None = None
    speed_percent: int | None = None
    direction: int | None = None
    duration_ms: int | None = None
    delay_after_ms: int = 0
    note: str = ""
    created_at_utc: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())


@dataclass(slots=True)
class SequenceSession:
    session_name: str
    created_at_utc: str
    updated_at_utc: str
    dume_base_url: str | None
    dume_resolution_source: str
    included_continuous_joint: str | None
    excluded_continuous_joints: list[str]
    steps: list[SequenceStep] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["steps"] = [asdict(step) for step in self.steps]
        return payload

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "SequenceSession":
        return cls(
            session_name=str(payload["session_name"]),
            created_at_utc=str(payload["created_at_utc"]),
            updated_at_utc=str(payload["updated_at_utc"]),
            dume_base_url=payload.get("dume_base_url"),
            dume_resolution_source=str(payload.get("dume_resolution_source", "unknown")),
            included_continuous_joint=payload.get("included_continuous_joint"),
            excluded_continuous_joints=[str(item) for item in payload.get("excluded_continuous_joints", [])],
            steps=[SequenceStep(**item) for item in payload.get("steps", [])],
        )


@dataclass(slots=True)
class ExecutedSequenceCommand:
    endpoint: str
    params: dict[str, Any]
    step_kind: str
    step_note: str
    device_response: dict[str, Any] | None = None


@dataclass(slots=True)
class SequenceExecutionResult:
    session_name: str
    mode: str
    dry_run: bool
    executed_commands: list[ExecutedSequenceCommand] = field(default_factory=list)
    reverse_continuous_steps: list[SequenceStep] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "session_name": self.session_name,
            "mode": self.mode,
            "dry_run": self.dry_run,
            "executed_commands": [asdict(item) for item in self.executed_commands],
            "reverse_continuous_steps": [asdict(item) for item in self.reverse_continuous_steps],
        }


class DumeSequenceModule:
    def __init__(self, root: Path, config: dict) -> None:
        self.root = root
        self.config = config
        self.output_dir = root / "output"
        self.sessions_dir = self.output_dir / "sequences"
        self.sessions_dir.mkdir(parents=True, exist_ok=True)

    def create_session(self, session_name: str) -> SequenceSession:
        if self.session_path(session_name).exists():
            raise FileExistsError(f"Sequence session already exists: {session_name}")

        base_url = self._configured_base_url()
        source = "config" if base_url else "unresolved"
        try:
            live_state = self._fetch_device_state(base_url)
            included_continuous_joint, excluded_continuous_joints = self._select_continuous_joint_policy(live_state)
        except Exception:
            included_continuous_joint = "wrist_rotate"
            excluded_continuous_joints = ["wrist_pitch"]
            source = f"{source}_offline_fallback"

        timestamp = datetime.now(timezone.utc).isoformat()
        session = SequenceSession(
            session_name=session_name,
            created_at_utc=timestamp,
            updated_at_utc=timestamp,
            dume_base_url=base_url,
            dume_resolution_source=source,
            included_continuous_joint=included_continuous_joint,
            excluded_continuous_joints=excluded_continuous_joints,
            steps=[],
        )
        self.save_session(session)
        return session

    def load_session(self, session_name: str) -> SequenceSession:
        path = self.session_path(session_name)
        if not path.exists():
            raise FileNotFoundError(f"Unknown sequence session: {session_name}")
        return SequenceSession.from_dict(json.loads(path.read_text(encoding="utf-8")))

    def save_session(self, session: SequenceSession) -> Path:
        session.updated_at_utc = datetime.now(timezone.utc).isoformat()
        path = self.session_path(session.session_name)
        path.write_text(json.dumps(session.to_dict(), indent=2), encoding="utf-8")
        return path

    def replay(self, session_name: str, *, dry_run: bool | None = None) -> SequenceExecutionResult:
        session = self.load_session(session_name)
        dry = self._resolve_dry_run(dry_run)
        result = SequenceExecutionResult(session_name=session_name, mode="replay", dry_run=dry)
        for step in session.steps:
            result.executed_commands.extend(self._execute_step(session, step, dry_run=dry))
        return result

    def return_home(self, session_name: str, *, dry_run: bool | None = None) -> SequenceExecutionResult:
        session = self.load_session(session_name)
        dry = self._resolve_dry_run(dry_run)
        result = SequenceExecutionResult(session_name=session_name, mode="return_home", dry_run=dry)

        home_command = ExecutedSequenceCommand(
            endpoint="/api/system",
            params={"cmd": "home_all"},
            step_kind="home_all",
            step_note="Home all joints before reversing the included 360 joint history.",
        )
        if not dry:
            home_command.device_response = self._run_command(session.dume_base_url, home_command.endpoint, home_command.params)
        result.executed_commands.append(home_command)

        reverse_steps = self._reverse_continuous_steps(session)
        result.reverse_continuous_steps = reverse_steps
        for step in reverse_steps:
            result.executed_commands.extend(self._execute_step(session, step, dry_run=dry))
        return result

    def session_path(self, session_name: str) -> Path:
        return self.sessions_dir / f"{session_name}.json"

    def _configured_base_url(self) -> str | None:
        bridge = self.config.get("bridge", {})
        if not isinstance(bridge, dict):
            return None
        base_url = bridge.get("device_base_url")
        return str(base_url).rstrip("/") if base_url else None

    def _resolve_dry_run(self, dry_run: bool | None) -> bool:
        if dry_run is not None:
            return bool(dry_run)
        bridge = self.config.get("bridge", {})
        return bool(bridge.get("dry_run", True))

    def _run_command(self, base_url: str | None, endpoint: str, params: dict[str, Any]) -> dict[str, Any]:
        if not base_url:
            raise RuntimeError("No biped robot base URL resolved for sequence module.")
        response = requests.get(f"{base_url.rstrip('/')}{endpoint}", params=params, timeout=8)
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", True):
            raise RuntimeError(payload.get("error", "device_request_failed"))
        return payload

    def _execute_step(self, session: SequenceSession, step: SequenceStep, *, dry_run: bool) -> list[ExecutedSequenceCommand]:
        commands: list[ExecutedSequenceCommand] = []

        if step.kind == "positional_move":
            command = ExecutedSequenceCommand(
                endpoint="/api/joint",
                params={"cmd": "move", "joint": step.joint_name, "value": int(step.target_value)},
                step_kind=step.kind,
                step_note=step.note,
            )
            if not dry_run:
                command.device_response = self._run_command(session.dume_base_url, command.endpoint, command.params)
                self._sleep_ms(step.delay_after_ms)
            commands.append(command)
            return commands

        if step.kind == "continuous_burst":
            signed_speed = int(step.speed_percent) * int(step.direction)
            start_command = ExecutedSequenceCommand(
                endpoint="/api/joint",
                params={"cmd": "move", "joint": step.joint_name, "value": signed_speed},
                step_kind=step.kind,
                step_note=step.note or "Start continuous burst",
            )
            stop_command = ExecutedSequenceCommand(
                endpoint="/api/joint",
                params={"cmd": "move", "joint": step.joint_name, "value": 0},
                step_kind=f"{step.kind}_stop",
                step_note=f"Stop {step.joint_name} after {step.duration_ms} ms",
            )
            if not dry_run:
                start_command.device_response = self._run_command(session.dume_base_url, start_command.endpoint, start_command.params)
                self._sleep_ms(step.duration_ms)
                stop_command.device_response = self._run_command(session.dume_base_url, stop_command.endpoint, stop_command.params)
                self._sleep_ms(step.delay_after_ms)
            commands.extend([start_command, stop_command])
            return commands

        raise ValueError(f"Unsupported sequence step kind: {step.kind}")

    def _reverse_continuous_steps(self, session: SequenceSession) -> list[SequenceStep]:
        reverse_steps: list[SequenceStep] = []
        for step in reversed(session.steps):
            if step.kind != "continuous_burst":
                continue
            if step.joint_name != session.included_continuous_joint:
                continue
            reverse_steps.append(
                SequenceStep(
                    kind="continuous_burst",
                    joint_name=step.joint_name,
                    speed_percent=step.speed_percent,
                    direction=-int(step.direction),
                    duration_ms=step.duration_ms,
                    delay_after_ms=step.delay_after_ms,
                    note=f"Reverse of recorded burst: {step.note}".strip(),
                )
            )
        return reverse_steps

    def _fetch_device_state(self, base_url: str | None) -> dict[str, Any]:
        if not base_url:
            raise RuntimeError("Could not resolve biped robot endpoint to create sequence session.")
        response = requests.get(f"{base_url.rstrip('/')}/api/state", timeout=8)
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", True):
            raise RuntimeError(payload.get("error", "state_request_failed"))
        return payload

    def _select_continuous_joint_policy(self, payload: dict[str, Any]) -> tuple[str | None, list[str]]:
        continuous_joints = [
            str(joint.get("name", ""))
            for joint in payload.get("joints", [])
            if str(joint.get("motor_type", "")) == "continuous_360"
        ]
        excluded = ["wrist_pitch"] if "wrist_pitch" in continuous_joints else []
        included = [joint for joint in continuous_joints if joint not in excluded]
        return (included[0] if included else None, excluded)

    @staticmethod
    def _sleep_ms(duration_ms: int | None) -> None:
        if not duration_ms:
            return
        time.sleep(max(0.0, float(duration_ms) / 1000.0))
