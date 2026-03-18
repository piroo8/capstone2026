"""Utility helpers shared by the parking mission nodes.

These helpers keep the first-pass mission node and the revisit mission node
consistent in how they handle poses, yaw, JSON logs, and small temporary
avoidance targets.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from geometry_msgs.msg import Pose, Quaternion


@dataclass
class TemporaryTarget:
    """A short-lived setpoint used for sidestep or back-up behavior."""

    x: float
    y: float
    z: float
    yaw: float
    created_time: float


def ensure_parent_dir(path: Path) -> None:
    """Create the parent directory for a log file if it does not exist yet."""

    path.parent.mkdir(parents=True, exist_ok=True)


def write_json(path: Path, payload: Any) -> None:
    """Write a JSON payload with stable formatting for easy inspection."""

    ensure_parent_dir(path)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def read_json(path: Path, default: Any) -> Any:
    """Load a JSON file, or return the provided default if the file is missing."""

    if not path.exists():
        return default

    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except json.JSONDecodeError:
        return default


def copy_pose(src: Pose) -> Pose:
    """Make a deep-enough copy of a ROS pose for local mission state."""

    pose = Pose()
    pose.position.x = src.position.x
    pose.position.y = src.position.y
    pose.position.z = src.position.z
    pose.orientation = copy_quaternion(src.orientation)
    return pose


def copy_quaternion(src: Quaternion) -> Quaternion:
    """Copy a quaternion so later edits do not mutate the original message."""

    q = Quaternion()
    q.x = src.x
    q.y = src.y
    q.z = src.z
    q.w = src.w
    return q


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a flat-yaw quaternion for MAVROS position setpoints."""

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw from a quaternion using the standard ENU formula."""

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi] so yaw comparisons stay numerically stable."""

    return math.atan2(math.sin(angle), math.cos(angle))


def serialize_waypoints(waypoints: list[tuple[float, float, float]]) -> list[dict[str, Any]]:
    """Convert the hardcoded waypoint list into JSON-safe dictionaries."""

    return [
        {
            'waypoint_index': index + 1,
            'position': [float(x), float(y), float(z)],
        }
        for index, (x, y, z) in enumerate(waypoints)
    ]
