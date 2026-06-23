"""
Geometric Inverse Kinematics solver for a 4-DOF revolute arm.

Joint configuration:
  J1 — base rotation (yaw)
  J2 — shoulder (pitch)
  J3 — elbow (pitch)
  J4 — wrist (pitch)

All angles in radians. Link lengths in meters.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class ArmConfig:
    l1: float = 0.105   # shoulder to elbow (m)
    l2: float = 0.096   # elbow to wrist (m)
    l3: float = 0.070   # wrist to end-effector (m)
    base_height: float = 0.085  # base to shoulder joint (m)

    # Joint limits (radians)
    j1_lim: Tuple[float, float] = (-np.pi, np.pi)
    j2_lim: Tuple[float, float] = (-np.pi / 2, np.pi / 2)
    j3_lim: Tuple[float, float] = (-np.pi / 2, np.pi / 2)
    j4_lim: Tuple[float, float] = (-np.pi / 2, np.pi / 2)


class IKSolver:
    """
    Analytical geometric IK for 4-DOF arm.

    Decouples the problem:
      1. J1 (base yaw) from target x, y in ground plane
      2. J2, J3, J4 (planar 3-link) in the vertical plane
         defined by J1
    """

    def __init__(self, config: ArmConfig = None):
        self.cfg = config or ArmConfig()

    def solve(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        end_effector_pitch: float = 0.0,
    ) -> Optional[np.ndarray]:
        """
        Solve IK for target position (x, y, z) in robot base frame.

        Args:
            target_x, target_y, target_z: target in base frame (m)
            end_effector_pitch: desired wrist pitch relative to horizontal (rad)

        Returns:
            np.ndarray of [j1, j2, j3, j4] in radians, or None if unreachable
        """
        cfg = self.cfg

        # --- J1: base rotation ---
        j1 = np.arctan2(target_y, target_x)

        # Planar distance from base axis to target (in J1 plane)
        r = np.sqrt(target_x**2 + target_y**2)

        # Height above shoulder joint
        z_rel = target_z - cfg.base_height

        # Wrist position: back off end-effector length along approach vector
        approach_angle = end_effector_pitch
        wx = r - cfg.l3 * np.cos(approach_angle)
        wz = z_rel - cfg.l3 * np.sin(approach_angle)

        # Distance from shoulder to wrist
        d = np.sqrt(wx**2 + wz**2)

        # Reachability check
        if d > (cfg.l1 + cfg.l2) or d < abs(cfg.l1 - cfg.l2):
            return None  # target unreachable

        # --- J3: elbow angle (law of cosines) ---
        cos_j3 = (d**2 - cfg.l1**2 - cfg.l2**2) / (2 * cfg.l1 * cfg.l2)
        cos_j3 = np.clip(cos_j3, -1.0, 1.0)
        j3 = -np.arccos(cos_j3)  # elbow-down configuration

        # --- J2: shoulder angle ---
        alpha = np.arctan2(wz, wx)
        beta = np.arctan2(cfg.l2 * np.sin(-j3), cfg.l1 + cfg.l2 * np.cos(-j3))
        j2 = alpha - beta

        # --- J4: wrist pitch to maintain desired end-effector angle ---
        j4 = end_effector_pitch - j2 - j3

        joints = np.array([j1, j2, j3, j4])

        # Joint limit check
        limits = [cfg.j1_lim, cfg.j2_lim, cfg.j3_lim, cfg.j4_lim]
        for i, (angle, (lo, hi)) in enumerate(zip(joints, limits)):
            if not (lo <= angle <= hi):
                return None  # joint limit violated

        return joints

    def forward_kinematics(self, joints: np.ndarray) -> np.ndarray:
        """
        Forward kinematics for verification.
        Returns end-effector position [x, y, z] in base frame.
        """
        cfg = self.cfg
        j1, j2, j3, j4 = joints

        # Cumulative angle in vertical plane
        a2 = j2
        a3 = j2 + j3
        a4 = j2 + j3 + j4

        r = (cfg.l1 * np.cos(a2)
             + cfg.l2 * np.cos(a3)
             + cfg.l3 * np.cos(a4))
        z = (cfg.base_height
             + cfg.l1 * np.sin(a2)
             + cfg.l2 * np.sin(a3)
             + cfg.l3 * np.sin(a4))

        x = r * np.cos(j1)
        y = r * np.sin(j1)

        return np.array([x, y, z])


if __name__ == "__main__":
    solver = IKSolver()

    targets = [
        (0.20, 0.10, 0.15, 0.0),
        (0.15, 0.00, 0.05, -0.3),
        (0.10, 0.10, 0.20, 0.0),
    ]

    for tx, ty, tz, pitch in targets:
        joints = solver.solve(tx, ty, tz, pitch)
        if joints is not None:
            fk = solver.forward_kinematics(joints)
            err = np.linalg.norm(np.array([tx, ty, tz]) - fk)
            print(f"Target ({tx:.2f},{ty:.2f},{tz:.2f}) → "
                  f"joints={np.degrees(joints).round(1)} deg | "
                  f"FK error={err*1000:.2f} mm")
        else:
            print(f"Target ({tx:.2f},{ty:.2f},{tz:.2f}) → UNREACHABLE")
