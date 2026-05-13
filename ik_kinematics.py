from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class JointLimits:
    minimum: float
    maximum: float

    def clamp(self, value: float) -> float:
        return max(self.minimum, min(self.maximum, value))

    def contains(self, value: float) -> bool:
        return self.minimum <= value <= self.maximum


@dataclass(frozen=True)
class JointSolution:
    base_deg: float
    shoulder_deg: float
    elbow_deg: float


@dataclass(frozen=True)
class ServoPose:
    base: int
    shoulder: int
    elbow: int


@dataclass(frozen=True)
class ArmGeometry:
    base_height_mm: float
    shoulder_to_elbow_mm: float
    elbow_to_wrist_mm: float
    wrist_to_gripper_tip_mm: float
    platform_width_mm: float
    platform_depth_mm: float
    platform_safe_clearance_mm: float
    base_limits_deg: JointLimits
    shoulder_limits_deg: JointLimits
    elbow_limits_deg: JointLimits
    servo_zero_deg: dict[str, float]
    servo_direction: dict[str, float]


def _deg(value: float) -> float:
    return math.degrees(value)


def _rad(value: float) -> float:
    return math.radians(value)


def load_geometry(config_path: Path) -> ArmGeometry:
    try:
        payload = json.loads(config_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            f"Invalid IK geometry config at {config_path}: {exc.msg} "
            f"(line {exc.lineno}, column {exc.colno})."
        ) from exc
    lengths = payload["link_lengths_mm"]
    limits = payload["joint_limits_deg"]
    mapping = payload.get("servo_mapping", {})
    platform = payload.get("platform", {})
    zeros = mapping.get("zero_deg", {})
    directions = mapping.get("direction", {})
    return ArmGeometry(
        base_height_mm=float(lengths["base_height"]),
        shoulder_to_elbow_mm=float(lengths["shoulder_to_elbow"]),
        elbow_to_wrist_mm=float(lengths["elbow_to_wrist"]),
        wrist_to_gripper_tip_mm=float(lengths["wrist_to_gripper_tip"]),
        platform_width_mm=float(platform.get("width_mm", 240.0)),
        platform_depth_mm=float(platform.get("depth_mm", 240.0)),
        platform_safe_clearance_mm=float(platform.get("safe_clearance_mm", 5.0)),
        base_limits_deg=JointLimits(*map(float, limits["base"])),
        shoulder_limits_deg=JointLimits(*map(float, limits["shoulder"])),
        elbow_limits_deg=JointLimits(*map(float, limits["elbow"])),
        servo_zero_deg={
            "base": float(zeros.get("base", 90.0)),
            "shoulder": float(zeros.get("shoulder", 90.0)),
            "elbow": float(zeros.get("elbow", 90.0)),
        },
        servo_direction={
            "base": float(directions.get("base", 1.0)),
            "shoulder": float(directions.get("shoulder", 1.0)),
            "elbow": float(directions.get("elbow", 1.0)),
        },
    )


def servo_to_mechanical(geometry: ArmGeometry, pose: ServoPose) -> JointSolution:
    return JointSolution(
        base_deg=(pose.base - geometry.servo_zero_deg["base"]) * geometry.servo_direction["base"],
        shoulder_deg=(pose.shoulder - geometry.servo_zero_deg["shoulder"]) * geometry.servo_direction["shoulder"],
        elbow_deg=(pose.elbow - geometry.servo_zero_deg["elbow"]) * geometry.servo_direction["elbow"],
    )


def mechanical_to_servo(geometry: ArmGeometry, solution: JointSolution) -> ServoPose:
    return ServoPose(
        base=int(round((solution.base_deg / geometry.servo_direction["base"]) + geometry.servo_zero_deg["base"])),
        shoulder=int(round((solution.shoulder_deg / geometry.servo_direction["shoulder"]) + geometry.servo_zero_deg["shoulder"])),
        elbow=int(round((solution.elbow_deg / geometry.servo_direction["elbow"]) + geometry.servo_zero_deg["elbow"])),
    )


def forward_kinematics(geometry: ArmGeometry, solution: JointSolution) -> Pose3D:
    base = _rad(solution.base_deg)
    shoulder = _rad(solution.shoulder_deg)
    elbow = _rad(solution.elbow_deg)

    l1 = geometry.shoulder_to_elbow_mm
    l2 = geometry.elbow_to_wrist_mm + geometry.wrist_to_gripper_tip_mm

    planar_r = l1 * math.cos(shoulder) + l2 * math.cos(shoulder + elbow)
    z = geometry.base_height_mm + l1 * math.sin(shoulder) + l2 * math.sin(shoulder + elbow)
    x = planar_r * math.cos(base)
    y = planar_r * math.sin(base)
    return Pose3D(x=x, y=y, z=z)


def inverse_kinematics(
    geometry: ArmGeometry,
    target: Pose3D,
    *,
    elbow_up: bool = False,
) -> JointSolution:
    base_deg = _deg(math.atan2(target.y, target.x))
    radial = math.hypot(target.x, target.y)
    planar_z = target.z - geometry.base_height_mm

    l1 = geometry.shoulder_to_elbow_mm
    l2 = geometry.elbow_to_wrist_mm + geometry.wrist_to_gripper_tip_mm
    radius_sq = (radial * radial) + (planar_z * planar_z)
    cos_elbow = (radius_sq - (l1 * l1) - (l2 * l2)) / (2.0 * l1 * l2)

    if cos_elbow < -1.0 or cos_elbow > 1.0:
        raise ValueError("Target is outside the reachable workspace.")

    elbow = math.acos(max(-1.0, min(1.0, cos_elbow)))
    if elbow_up:
        elbow = -elbow

    k1 = l1 + l2 * math.cos(elbow)
    k2 = l2 * math.sin(elbow)
    shoulder = math.atan2(planar_z, radial) - math.atan2(k2, k1)

    shoulder_deg = _deg(shoulder)
    elbow_deg = _deg(elbow)

    if not geometry.base_limits_deg.contains(base_deg):
        raise ValueError(f"Base solution {base_deg:.2f} deg exceeds limits.")
    if not geometry.shoulder_limits_deg.contains(shoulder_deg):
        raise ValueError(f"Shoulder solution {shoulder_deg:.2f} deg exceeds limits.")
    if not geometry.elbow_limits_deg.contains(elbow_deg):
        raise ValueError(f"Elbow solution {elbow_deg:.2f} deg exceeds limits.")

    return JointSolution(
        base_deg=base_deg,
        shoulder_deg=shoulder_deg,
        elbow_deg=elbow_deg,
    )
