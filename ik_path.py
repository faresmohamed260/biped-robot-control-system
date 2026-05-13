from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import requests

from ik_kinematics import (
    ArmGeometry,
    JointSolution,
    Pose3D,
    ServoPose,
    forward_kinematics,
    inverse_kinematics,
    load_geometry,
    mechanical_to_servo,
    servo_to_mechanical,
)


IK_JOINTS = ("base", "shoulder", "elbow")
GRIPPER_ACTIONS = ("hold", "open", "close")


@dataclass(slots=True)
class CapturedIkPose:
    captured_at_utc: str
    servo_pose: dict[str, int]
    cartesian_pose_mm: dict[str, float]


@dataclass(slots=True)
class IkPathPlan:
    name: str
    created_at_utc: str
    start_pose: CapturedIkPose
    end_pose: CapturedIkPose
    interpolation_steps: int
    delay_after_ms: int
    use_safe_height: bool
    safe_height_mm: float
    gripper_prepare_action: str = "open"
    gripper_action_after_start_delay: str = "close"
    gripper_action_at_end: str = "open"
    gripper_start_delay_ms: int = 300
    gripper_end_delay_ms: int = 0
    gripper_open_value: int | None = None
    gripper_close_value: int | None = None
    start_target_command_index: int = 0
    end_target_command_index: int = 0
    waypoints_mm: list[dict[str, float]] = field(default_factory=list)
    commands: list[dict[str, int]] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


class IkPathModule:
    def __init__(self, root: Path, base_url: str) -> None:
        self.root = root
        self.base_url = base_url.rstrip("/")
        self.geometry = load_geometry(root / "config" / "arm_geometry.json")
        self.output_dir = root / "output" / "ik_paths"
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def capture_pose(self) -> CapturedIkPose:
        payload = self._fetch_state()
        servo_pose = {
            joint["name"]: int(joint["position"])
            for joint in payload.get("joints", [])
            if joint.get("name") in IK_JOINTS
        }
        missing = [name for name in IK_JOINTS if name not in servo_pose]
        if missing:
            raise RuntimeError(f"Missing joints in live state: {', '.join(missing)}")
        mechanical = servo_to_mechanical(
            self.geometry,
            ServoPose(
                base=servo_pose["base"],
                shoulder=servo_pose["shoulder"],
                elbow=servo_pose["elbow"],
            ),
        )
        xyz = forward_kinematics(self.geometry, mechanical)
        return CapturedIkPose(
            captured_at_utc=datetime.now(timezone.utc).isoformat(),
            servo_pose=servo_pose,
            cartesian_pose_mm={"x": xyz.x, "y": xyz.y, "z": xyz.z},
        )

    def build_plan(
        self,
        name: str,
        start_pose: CapturedIkPose,
        end_pose: CapturedIkPose,
        *,
        interpolation_steps: int,
        delay_after_ms: int,
        use_safe_height: bool,
        safe_height_mm: float,
        gripper_prepare_action: str = "open",
        gripper_action_after_start_delay: str = "close",
        gripper_action_at_end: str = "hold",
        gripper_start_delay_ms: int = 300,
        gripper_end_delay_ms: int = 0,
        gripper_close_value: int | None = None,
    ) -> IkPathPlan:
        steps = max(2, int(interpolation_steps))
        delay = max(0, int(delay_after_ms))
        safe_height = float(safe_height_mm)
        prepare_action = self._normalize_gripper_action(gripper_prepare_action)
        start_action = self._normalize_gripper_action(gripper_action_after_start_delay)
        end_action = self._normalize_gripper_action(gripper_action_at_end)
        start_delay = max(0, int(gripper_start_delay_ms))
        end_delay = max(0, int(gripper_end_delay_ms))
        gripper_targets = (
            self._fetch_gripper_targets(gripper_close_value)
            if (prepare_action != "hold" or start_action != "hold" or end_action != "hold" or gripper_close_value is not None)
            else {"open": None, "close": None, "minimum": None, "maximum": None}
        )

        start_xyz = Pose3D(**start_pose.cartesian_pose_mm)
        end_xyz = Pose3D(**end_pose.cartesian_pose_mm)

        waypoint_sequence: list[Pose3D]
        if use_safe_height:
            start_lift = Pose3D(start_xyz.x, start_xyz.y, max(start_xyz.z, safe_height))
            end_lift = Pose3D(end_xyz.x, end_xyz.y, max(end_xyz.z, safe_height))
            waypoint_sequence = [start_lift, start_xyz, start_lift, end_lift, end_xyz]
            start_target_index = 1
            end_target_index = len(waypoint_sequence) - 1
        else:
            waypoint_sequence = [start_xyz, end_xyz]
            start_target_index = 0
            end_target_index = len(waypoint_sequence) - 1

        cartesian_points = self._interpolate_waypoints(waypoint_sequence, steps)
        for point in cartesian_points:
            self._validate_workspace_point(point)
        commands: list[dict[str, int]] = []
        start_target_command_index = 0
        end_target_command_index = 0
        for point in cartesian_points:
            solution = inverse_kinematics(self.geometry, point)
            servo_pose = mechanical_to_servo(self.geometry, solution)
            command = {
                "base": servo_pose.base,
                "shoulder": servo_pose.shoulder,
                "elbow": servo_pose.elbow,
            }
            if not commands or command != commands[-1]:
                commands.append(command)
            current_index = len(commands) - 1
            if self._points_match(point, waypoint_sequence[start_target_index]):
                start_target_command_index = current_index
            if self._points_match(point, waypoint_sequence[end_target_index]):
                end_target_command_index = current_index

        plan = IkPathPlan(
            name=name,
            created_at_utc=datetime.now(timezone.utc).isoformat(),
            start_pose=start_pose,
            end_pose=end_pose,
            interpolation_steps=steps,
            delay_after_ms=delay,
            use_safe_height=bool(use_safe_height),
            safe_height_mm=safe_height,
            gripper_prepare_action=prepare_action,
            gripper_action_after_start_delay=start_action,
            gripper_action_at_end=end_action,
            gripper_start_delay_ms=start_delay,
            gripper_end_delay_ms=end_delay,
            gripper_open_value=gripper_targets["open"],
            gripper_close_value=gripper_targets["close"],
            start_target_command_index=start_target_command_index,
            end_target_command_index=end_target_command_index,
            waypoints_mm=[{"x": point.x, "y": point.y, "z": point.z} for point in cartesian_points],
            commands=commands,
        )
        self.save_plan(plan)
        return plan

    def execute_plan(self, plan: IkPathPlan, *, dry_run: bool) -> dict:
        executed: list[dict] = []
        prepare_gripper = self._build_gripper_command(plan.gripper_prepare_action, plan)
        if prepare_gripper is not None:
            response = None
            if not dry_run:
                response = self._run_command("/api/joint", prepare_gripper["params"])
            executed.append(
                {
                    "joint": "gripper",
                    "action": plan.gripper_prepare_action,
                    "value": prepare_gripper["params"]["value"],
                    "device_response": response,
                }
            )
            if not dry_run and plan.delay_after_ms > 0:
                time.sleep(plan.delay_after_ms / 1000.0)

        for index, command in enumerate(plan.commands):
            for joint in IK_JOINTS:
                params = {"cmd": "move", "joint": joint, "value": int(command[joint])}
                response = None
                if not dry_run:
                    response = self._run_command("/api/joint", params)
                executed.append(
                    {
                        "joint": joint,
                        "value": int(command[joint]),
                        "device_response": response,
                    }
                )
            if not dry_run and plan.delay_after_ms > 0:
                time.sleep(plan.delay_after_ms / 1000.0)
            if index == plan.start_target_command_index:
                if not dry_run and plan.gripper_start_delay_ms > 0:
                    time.sleep(plan.gripper_start_delay_ms / 1000.0)
                start_gripper = self._build_gripper_command(plan.gripper_action_after_start_delay, plan)
                if start_gripper is not None:
                    response = None
                    if not dry_run:
                        response = self._run_command("/api/joint", start_gripper["params"])
                    executed.append(
                        {
                            "joint": "gripper",
                            "action": plan.gripper_action_after_start_delay,
                            "value": start_gripper["params"]["value"],
                            "device_response": response,
                        }
                    )
                    if not dry_run and plan.delay_after_ms > 0:
                        time.sleep(plan.delay_after_ms / 1000.0)

            if index == plan.end_target_command_index:
                end_gripper = self._build_gripper_command(plan.gripper_action_at_end, plan)
                if end_gripper is not None:
                    if not dry_run and plan.gripper_end_delay_ms > 0:
                        time.sleep(plan.gripper_end_delay_ms / 1000.0)
                    response = None
                    if not dry_run:
                        response = self._run_command("/api/joint", end_gripper["params"])
                    executed.append(
                        {
                            "joint": "gripper",
                            "action": plan.gripper_action_at_end,
                            "value": end_gripper["params"]["value"],
                            "device_response": response,
                        }
                    )
        return {
            "plan_name": plan.name,
            "dry_run": dry_run,
            "commands": executed,
        }

    def save_plan(self, plan: IkPathPlan) -> Path:
        path = self.output_dir / f"{plan.name}.json"
        path.write_text(json.dumps(plan.to_dict(), indent=2), encoding="utf-8")
        return path

    def _interpolate_waypoints(self, waypoints: list[Pose3D], steps_per_segment: int) -> list[Pose3D]:
        points: list[Pose3D] = []
        for index in range(len(waypoints) - 1):
            start = waypoints[index]
            end = waypoints[index + 1]
            for step in range(1, steps_per_segment + 1):
                alpha = step / float(steps_per_segment)
                point = Pose3D(
                    x=start.x + (end.x - start.x) * alpha,
                    y=start.y + (end.y - start.y) * alpha,
                    z=start.z + (end.z - start.z) * alpha,
                )
                if not points or point != points[-1]:
                    points.append(point)
        return points

    def _validate_workspace_point(self, point: Pose3D) -> None:
        half_width = self.geometry.platform_width_mm / 2.0
        half_depth = self.geometry.platform_depth_mm / 2.0
        inside_platform = (-half_width <= point.x <= half_width) and (-half_depth <= point.y <= half_depth)
        if inside_platform and point.z < self.geometry.platform_safe_clearance_mm:
            raise ValueError(
                "IK waypoint would intersect the platform top. "
                f"Point ({point.x:.1f}, {point.y:.1f}, {point.z:.1f}) mm is inside the "
                f"{self.geometry.platform_width_mm:.0f}x{self.geometry.platform_depth_mm:.0f} mm platform footprint "
                f"but below the safe clearance of {self.geometry.platform_safe_clearance_mm:.1f} mm."
            )

    def _points_match(self, left: Pose3D, right: Pose3D, tolerance_mm: float = 1e-6) -> bool:
        return (
            abs(left.x - right.x) <= tolerance_mm
            and abs(left.y - right.y) <= tolerance_mm
            and abs(left.z - right.z) <= tolerance_mm
        )

    def _fetch_state(self) -> dict:
        response = requests.get(f"{self.base_url}/api/state", timeout=8)
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", True):
            raise RuntimeError(payload.get("error", "state_request_failed"))
        return payload

    def _fetch_gripper_targets(self, close_override: int | None = None) -> dict[str, int]:
        payload = self._fetch_state()
        for joint in payload.get("joints", []):
            if joint.get("name") == "gripper":
                minimum = int(joint.get("min_angle", joint.get("stored_min_angle", 20)))
                maximum = int(joint.get("max_angle", joint.get("stored_max_angle", 160)))
                close_value = minimum if close_override is None else max(minimum, min(maximum, int(close_override)))
                return {"open": maximum, "close": close_value, "minimum": minimum, "maximum": maximum}
        raise RuntimeError("Missing gripper joint in live state.")

    def _normalize_gripper_action(self, action: str) -> str:
        normalized = str(action or "hold").strip().lower()
        if normalized not in GRIPPER_ACTIONS:
            raise ValueError(f"Unsupported gripper action: {action}")
        return normalized

    def _build_gripper_command(self, action: str, plan: IkPathPlan) -> dict | None:
        if action == "hold":
            return None
        if action == "open":
            if plan.gripper_open_value is None:
                raise RuntimeError("Gripper open target is missing from the IK plan.")
            value = int(plan.gripper_open_value)
        elif action == "close":
            if plan.gripper_close_value is None:
                raise RuntimeError("Gripper close target is missing from the IK plan.")
            value = int(plan.gripper_close_value)
        else:
            raise ValueError(f"Unsupported gripper action: {action}")
        return {
            "params": {
                "cmd": "move",
                "joint": "gripper",
                "value": value,
            }
        }

    def _run_command(self, endpoint: str, params: dict) -> dict:
        response = requests.get(f"{self.base_url}{endpoint}", params=params, timeout=8)
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", True):
            raise RuntimeError(payload.get("error", "device_request_failed"))
        return payload
