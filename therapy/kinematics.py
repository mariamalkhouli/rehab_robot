# =============================================================================
#  therapy/kinematics.py
#  /home/pi/rehab_robot/therapy/kinematics.py
#
#  Forward & Inverse Kinematics Engine
#  4-DOF Lower Limb Rehabilitation Robot
#
# =============================================================================
#  COORDINATE SYSTEM
#  Origin: Hip joint center
#  X-axis: Points LATERALLY (away from body midline, toward right side)
#  Y-axis: Points SUPERIORLY (toward patient's head, along the bed surface)
#  Z-axis: Points ANTERIORLY (upward, away from the bed surface)
#
#  Patient is SUPINE (lying on back, face up).
#  All angles are in DEGREES throughout this module.
#  Positive angles follow the right-hand rule.
#
# =============================================================================
#  JOINT DEFINITIONS
#
#  Axis 1 — Hip Abduction / Adduction (motor 1)
#    Rotation about the Y-axis (leg swings sideways in the frontal plane)
#    θ₁ = 0°   → leg aligned with body midline
#    θ₁ > 0°   → ABDUCTION  (leg moves away from midline, toward outside)
#    θ₁ < 0°   → ADDUCTION  (leg moves toward midline, crossing)
#    Range: -30° to +30°
#
#    LEFT LEG MIRROR: For the left leg, abduction is in the negative X direction
#    so we apply the affected_side_sign multiplier (see below).
#
#  Axis 2 — Hip Flexion / Extension (motor 2)
#    Rotation about the X-axis (leg lifts off bed, swings in sagittal plane)
#    θ₂ = 0°   → leg flat on bed (full extension, home position)
#    θ₂ > 0°   → FLEXION  (leg lifts, knee comes toward chest)
#    θ₂ < 0°   → HYPEREXTENSION (leg pushes below bed plane — BLOCKED)
#    Range: 0° to 120°
#
#  Axis 3 — Knee Flexion / Extension (motor 3)
#    Rotation about the X-axis AT the knee joint
#    θ₃ = 0°   → knee fully extended (straight leg)
#    θ₃ > 0°   → FLEXION (heel comes toward buttocks)
#    Range: 0° to 130°
#
#  Axis 4 — Reserved / Internal-External Rotation (motor 4)
#    Currently placeholder. May be used for hip internal/external rotation.
#    θ₄ = 0°   → neutral rotation
#    Range: -45° to +45°
#
# =============================================================================
#  FORWARD KINEMATICS (FK)
#  Given joint angles → compute 3D positions of knee and cuff
#
#  With L1 = thigh length, L2 = shank length:
#
#  Hip joint (fixed at origin):
#    P_hip = [0, 0, 0]
#
#  Knee joint position:
#    P_knee.x = L1 * sin(θ₁)                              ← abduction offset
#    P_knee.y = -L1 * cos(θ₁) * cos(θ₂)                  ← along-bed component
#    P_knee.z = L1 * cos(θ₁) * sin(θ₂)                   ← lift component
#
#  Cuff position (end effector, at the leg cuff below knee):
#    The knee bends the shank by (θ₃) relative to the thigh direction.
#    Total shank angle from vertical = θ₂ - θ₃  (knee flex reduces elevation)
#
#    P_cuff.x = P_knee.x   (abduction doesn't change in sagittal motion)
#    P_cuff.y = P_knee.y - L2 * cos(θ₁) * cos(θ₂ - θ₃)
#    P_cuff.z = P_knee.z + L2 * cos(θ₁) * sin(θ₂ - θ₃)
#
# =============================================================================
#  EXERCISES AND MOTOR MAPPING
#
#  Exercise 1 — Passive CPM (Knee Flexion/Extension)
#    Primary: Motor 3 (knee): 0° → range_max → 0°
#    Secondary: Motor 2 (hip): follows at ~25% of knee angle to keep thigh stable
#    Motor 1, 4: stationary at 0°
#
#  Exercise 2 — Straight Leg Raise (Hip Flexion, knee locked straight)
#    Primary: Motor 2 (hip flex): 0° → 60–90° → 0°
#    Motor 3 (knee): stays at 0° (leg remains straight throughout)
#    Motor 1, 4: stationary at 0°
#
#  Exercise 3 — Hip + Knee Combined Flexion (bring knee to chest)
#    Motor 2 (hip): 0° → 90° (flexes)
#    Motor 3 (knee): 0° → 90° (flexes simultaneously)
#    Both move in coordination — this is the "cycling" motion
#    Motor 1, 4: stationary at 0°
#
#  Exercise 4 — Hip Abduction / Adduction
#    Motor 1 only: 0° → +30° (abduction) or 0° → -15° (adduction)
#    Motors 2, 3, 4: stationary at 0° (leg stays flat on bed)
#
#  Exercise 5 — Full Combined CPM (all joints)
#    Motor 2 (hip) + Motor 3 (knee): coordinated sinusoidal motion
#    Simulates natural gait-like leg cycling in the sagittal plane
#
# =============================================================================

import math
import numpy as np
import logging
from dataclasses import dataclass
from typing import List, Tuple, Optional

from core.config import CFG

logger = logging.getLogger(__name__)


# =============================================================================
#  Data Structures
# =============================================================================

@dataclass
class JointAngles:
    """4-DOF joint angle set. All values in degrees."""
    ax1: float = 0.0   # Hip Abduction/Adduction
    ax2: float = 0.0   # Hip Flexion/Extension
    ax3: float = 0.0   # Knee Flexion/Extension
    ax4: float = 0.0   # Reserved / Hip Rotation

    def to_list(self) -> List[float]:
        return [self.ax1, self.ax2, self.ax3, self.ax4]

    @classmethod
    def from_list(cls, lst: List[float]) -> 'JointAngles':
        return cls(
            ax1=lst[0] if len(lst) > 0 else 0.0,
            ax2=lst[1] if len(lst) > 1 else 0.0,
            ax3=lst[2] if len(lst) > 2 else 0.0,
            ax4=lst[3] if len(lst) > 3 else 0.0,
        )

    def __repr__(self):
        return (f"JointAngles(hip_ab={self.ax1:.1f}°, "
                f"hip_flex={self.ax2:.1f}°, "
                f"knee_flex={self.ax3:.1f}°, "
                f"ax4={self.ax4:.1f}°)")


@dataclass
class CartesianPoint:
    """3D point in the robot coordinate frame. All values in mm."""
    x: float = 0.0   # Lateral (+ = right/abduction direction)
    y: float = 0.0   # Superior (+ = toward patient's head)
    z: float = 0.0   # Anterior (+ = away from bed surface)

    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z]

    def distance_to(self, other: 'CartesianPoint') -> float:
        return math.sqrt(
            (self.x - other.x)**2 +
            (self.y - other.y)**2 +
            (self.z - other.z)**2
        )


@dataclass
class KinematicsResult:
    """Complete FK result — positions of all key points."""
    hip_pos:   CartesianPoint = None
    knee_pos:  CartesianPoint = None
    cuff_pos:  CartesianPoint = None
    angles:    JointAngles    = None
    valid:     bool           = True
    message:   str            = ""

    def __post_init__(self):
        if self.hip_pos  is None: self.hip_pos  = CartesianPoint(0, 0, 0)
        if self.knee_pos is None: self.knee_pos = CartesianPoint()
        if self.cuff_pos is None: self.cuff_pos = CartesianPoint()
        if self.angles   is None: self.angles   = JointAngles()


# =============================================================================
#  Exercise Presets
# =============================================================================

class ExercisePreset:
    """Named exercise configurations with their motor patterns."""

    HOME = JointAngles(ax1=0.0, ax2=0.0, ax3=0.0, ax4=0.0)

    @staticmethod
    def knee_cpm(flex_max_deg: float = 90.0) -> dict:
        """
        Passive CPM — knee flexion/extension.
        Motor 3 sweeps 0 → flex_max. Motor 2 follows at 20% to support thigh.
        """
        return {
            "name"         : "Knee CPM",
            "description"  : "Continuous passive knee flexion/extension",
            "targets"      : ["knee_flex_ext"],
            "start"        : JointAngles(0, 0, 0, 0),
            "end"          : JointAngles(0, flex_max_deg * 0.2, flex_max_deg, 0),
            "motor_pattern": "3+2_follow",
        }

    @staticmethod
    def straight_leg_raise(flex_max_deg: float = 60.0) -> dict:
        """
        Straight Leg Raise (SLR) — hip flexion with knee locked straight.
        Motor 2 sweeps 0 → flex_max. Motor 3 stays at 0°.
        """
        return {
            "name"         : "Straight Leg Raise",
            "description"  : "Hip flexion with knee extended (SLR)",
            "targets"      : ["hip_flex_ext"],
            "start"        : JointAngles(0, 0, 0, 0),
            "end"          : JointAngles(0, flex_max_deg, 0, 0),
            "motor_pattern": "2_only",
        }

    @staticmethod
    def hip_knee_flexion(
        hip_max_deg: float  = 90.0,
        knee_max_deg: float = 90.0
    ) -> dict:
        """
        Combined hip + knee flexion — bring knee toward chest.
        Both motors 2 and 3 flex simultaneously.
        """
        return {
            "name"         : "Hip + Knee Flexion",
            "description"  : "Combined flexion — knee to chest",
            "targets"      : ["hip_flex_ext", "knee_flex_ext"],
            "start"        : JointAngles(0, 0, 0, 0),
            "end"          : JointAngles(0, hip_max_deg, knee_max_deg, 0),
            "motor_pattern": "2_and_3",
        }

    @staticmethod
    def hip_abduction(abd_max_deg: float = 25.0) -> dict:
        """
        Hip abduction — leg moves away from midline.
        Motor 1 only. Leg stays flat on bed.
        """
        return {
            "name"         : "Hip Abduction",
            "description"  : "Lateral leg movement (abduction)",
            "targets"      : ["hip_ab_ad"],
            "start"        : JointAngles(0, 0, 0, 0),
            "end"          : JointAngles(abd_max_deg, 0, 0, 0),
            "motor_pattern": "1_only",
        }

    @staticmethod
    def hip_adduction(add_max_deg: float = 15.0) -> dict:
        """
        Hip adduction — leg moves toward/across midline.
        Motor 1 moves to negative angle. Leg stays flat.
        """
        return {
            "name"         : "Hip Adduction",
            "description"  : "Medial leg movement (adduction)",
            "targets"      : ["hip_ab_ad"],
            "start"        : JointAngles(0, 0, 0, 0),
            "end"          : JointAngles(-add_max_deg, 0, 0, 0),
            "motor_pattern": "1_only",
        }

    @staticmethod
    def full_cpm(
        hip_max_deg:  float = 60.0,
        knee_max_deg: float = 90.0
    ) -> dict:
        """
        Full coordinated CPM — hip and knee cycle together.
        Simulates a natural gait-like motion.
        """
        return {
            "name"         : "Full CPM",
            "description"  : "Hip + knee coordinated cycling",
            "targets"      : ["hip_flex_ext", "knee_flex_ext"],
            "start"        : JointAngles(0, 0, 0, 0),
            "end"          : JointAngles(0, hip_max_deg, knee_max_deg, 0),
            "motor_pattern": "coordinated",
        }


# =============================================================================
#  Kinematics Engine
# =============================================================================

class KinematicsEngine:
    """
    Forward and Inverse Kinematics for the 4-DOF rehab robot.

    All angles are in degrees. All positions are in mm.

    The engine is stateless — it takes angles and returns positions.
    No thread safety needed (pure math, no shared state).
    """

    def __init__(self):
        # Limb segment lengths from config (in mm)
        self.L1 = CFG.kinematics.thigh_length_mm   # Hip to knee
        self.L2 = CFG.kinematics.shank_length_mm   # Knee to cuff

        # Affected side: +1 for right leg, -1 for left leg
        # Applied to ax1 (abduction/adduction) so the direction is correct
        # for both legs without changing any other math.
        # This is set per session from patient.affected_side
        self.side_sign = 1   # Default right leg

        # Mechanical zero offsets (from config — set after physical calibration)
        self._offsets = JointAngles(
            ax1 = CFG.kinematics.ax1_mechanical_offset_deg,
            ax2 = CFG.kinematics.ax2_mechanical_offset_deg,
            ax3 = CFG.kinematics.ax3_mechanical_offset_deg,
            ax4 = CFG.kinematics.ax4_mechanical_offset_deg,
        )

        logger.info(
            f"KinematicsEngine initialized. "
            f"L1={self.L1}mm (thigh), L2={self.L2}mm (shank). "
            f"Offsets: {self._offsets}"
        )

    def set_affected_side(self, side: str):
        """
        Set the affected leg.

        Args:
            side: 'right' or 'left'

        For the LEFT leg, abduction direction is mirrored.
        All other axes remain the same.
        """
        if side.lower() == 'left':
            self.side_sign = -1
            logger.info("Kinematics: set for LEFT leg (ax1 mirrored).")
        else:
            self.side_sign = 1
            logger.info("Kinematics: set for RIGHT leg.")

    # =========================================================================
    #  Forward Kinematics
    # =========================================================================

    def forward_kinematics(self, angles: JointAngles) -> KinematicsResult:
        """
        Compute 3D positions of hip, knee, and cuff from joint angles.

        Args:
            angles: JointAngles in degrees

        Returns:
            KinematicsResult with hip_pos, knee_pos, cuff_pos
        """
        # Apply mechanical offsets
        θ1 = math.radians((angles.ax1 * self.side_sign) + self._offsets.ax1)
        θ2 = math.radians(angles.ax2 + self._offsets.ax2)
        θ3 = math.radians(angles.ax3 + self._offsets.ax3)

        # ── Hip joint (fixed at origin) ────────────────────────────────────
        P_hip = CartesianPoint(0.0, 0.0, 0.0)

        # ── Knee joint position ────────────────────────────────────────────
        # The thigh (L1) rotates about:
        #   - Y-axis by θ1 (abduction) → spreads leg laterally
        #   - X-axis by θ2 (hip flex)  → lifts leg off bed
        #
        # With both rotations combined (order: abduct first, then flex):
        #   Knee.x = L1 * sin(θ1)
        #   Knee.y = -L1 * cos(θ1) * cos(θ2)
        #   Knee.z = L1 * cos(θ1) * sin(θ2)
        knee_x = self.L1 * math.sin(θ1)
        knee_y = -self.L1 * math.cos(θ1) * math.cos(θ2)
        knee_z = self.L1 * math.cos(θ1) * math.sin(θ2)

        P_knee = CartesianPoint(knee_x, knee_y, knee_z)

        # ── Cuff position (end effector) ───────────────────────────────────
        # The shank (L2) bends at the knee by θ3 RELATIVE to the thigh.
        # The total elevation angle of the shank = θ2 - θ3
        # (knee flexion reduces the elevation of the shank below the thigh)
        #
        # The shank extends from the knee in the direction:
        #   shank_angle_from_bed = θ2 - θ3
        #
        # In the sagittal plane the shank adds:
        #   dy = -L2 * cos(θ1) * cos(θ2 - θ3)
        #   dz = L2 * cos(θ1) * sin(θ2 - θ3)
        #
        # The lateral component doesn't change with knee flex
        # (knee flex is purely in the sagittal plane)
        shank_angle = θ2 - θ3

        cuff_x = knee_x
        cuff_y = knee_y - self.L2 * math.cos(θ1) * math.cos(shank_angle)
        cuff_z = knee_z + self.L2 * math.cos(θ1) * math.sin(shank_angle)

        P_cuff = CartesianPoint(cuff_x, cuff_y, cuff_z)

        return KinematicsResult(
            hip_pos  = P_hip,
            knee_pos = P_knee,
            cuff_pos = P_cuff,
            angles   = angles,
            valid    = True,
        )

    def forward_kinematics_from_list(self, angles: List[float]) -> KinematicsResult:
        """Convenience wrapper — accepts [ax1, ax2, ax3, ax4] list."""
        return self.forward_kinematics(JointAngles.from_list(angles))

    # =========================================================================
    #  Trajectory Generation
    # =========================================================================

    def generate_trajectory(
        self,
        start:  JointAngles,
        end:    JointAngles,
        n_points: int = None,
        profile: str  = "trapezoidal"
    ) -> List[JointAngles]:
        """
        Generate a smooth joint-space trajectory from start to end.

        This is the core path generator used by all therapy modes.
        The Pi pre-computes the entire path as a list of waypoints,
        then sends them one by one to the Arduino at the configured rate.

        Args:
            start     : Starting joint angles
            end       : Target joint angles
            n_points  : Number of interpolation points
                        Default: from config (passive_mode.trajectory_points)
            profile   : Motion profile
                        "linear"       — constant speed (simple, less smooth)
                        "trapezoidal"  — ramp up, constant, ramp down (standard)
                        "cosine"       — S-curve via cosine interpolation (smoothest)

        Returns:
            List of JointAngles from start to end (inclusive)
        """
        if n_points is None:
            n_points = CFG.passive_mode.trajectory_points

        if n_points < 2:
            return [start, end]

        # Generate parameter t from 0 to 1
        if profile == "cosine":
            # Cosine gives smooth S-curve: starts and ends gently
            t_values = [
                0.5 * (1 - math.cos(math.pi * i / (n_points - 1)))
                for i in range(n_points)
            ]
        elif profile == "trapezoidal":
            # Ramp up for first 20%, constant for 60%, ramp down for last 20%
            ramp = 0.2
            t_values = []
            for i in range(n_points):
                t = i / (n_points - 1)
                if t < ramp:
                    # Ramp up: smooth from 0
                    t_scaled = (t / ramp) ** 2 * ramp / 2
                elif t > 1.0 - ramp:
                    # Ramp down: mirror of ramp up
                    t_from_end = (1.0 - t) / ramp
                    t_scaled   = 1.0 - (t_from_end ** 2 * ramp / 2)
                else:
                    # Constant speed region
                    t_scaled = ramp / 2 + (t - ramp)
                t_values.append(t_scaled)
        else:
            # Linear — equal spacing
            t_values = [i / (n_points - 1) for i in range(n_points)]

        # Interpolate each axis
        trajectory = []
        for t in t_values:
            waypoint = JointAngles(
                ax1 = start.ax1 + t * (end.ax1 - start.ax1),
                ax2 = start.ax2 + t * (end.ax2 - start.ax2),
                ax3 = start.ax3 + t * (end.ax3 - start.ax3),
                ax4 = start.ax4 + t * (end.ax4 - start.ax4),
            )
            # Clamp to safe limits at every point
            waypoint = self.clamp_to_safe_limits(waypoint)
            trajectory.append(waypoint)

        return trajectory

    def generate_cpm_cycle(
        self,
        exercise_preset: dict,
        n_points_per_sweep: int = None,
        profile: str = "cosine"
    ) -> List[JointAngles]:
        """
        Generate one complete CPM cycle (outward sweep + return sweep).

        Args:
            exercise_preset : Result of ExercisePreset.knee_cpm() etc.
            n_points_per_sweep: Points in each half of the cycle
            profile       : Motion profile

        Returns:
            Full cycle: [home → end → home]
            The therapy engine loops this list continuously.
        """
        start = exercise_preset["start"]
        end   = exercise_preset["end"]

        n = n_points_per_sweep or CFG.passive_mode.trajectory_points

        # Outward sweep: start → end
        outward = self.generate_trajectory(start, end, n, profile)

        # Return sweep: end → start
        inward  = self.generate_trajectory(end, start, n, profile)

        # Combine — avoid duplicate midpoint
        return outward + inward[1:]

    def fill_m4(self, angles: JointAngles) -> JointAngles:
            """
            Calculates Motor 4 to keep the end-effector parallel to the leg.
            Math: The shank link's angle relative to the bed is (ax2 - ax3).
            Motor 4 must rotate exactly opposite to this to remain 'level'.
            """
            # We use a compensation factor from config (usually 1.0)
            k = getattr(CFG.kinematics, 'cuff_compensation_factor', 1.0)
            
            # This keeps the cuff at a constant orientation relative to the bed
            # regardless of how high the hip is or how bent the knee is.
            angles.ax4 = -(angles.ax2 - angles.ax3) * k
            
            return angles

    def generate_coordinated_cpm(
        self,
        hip_max_deg: float  = 60.0,
        knee_max_deg: float = 90.0,
        n_points: int       = None,
    ) -> List[JointAngles]:
        """
        Generate coordinated hip + knee CPM trajectory.

        The hip and knee flex and extend together in a natural gait-like pattern.
        The knee leads by 30° phase to simulate the natural leg swing.

        Args:
            hip_max_deg  : Maximum hip flexion (degrees)
            knee_max_deg : Maximum knee flexion (degrees)
            n_points     : Points per full cycle

        Returns:
            One complete cycle as a list of JointAngles
        """
        n = n_points or CFG.passive_mode.trajectory_points * 2

        trajectory = []
        for i in range(n):
            t     = i / (n - 1)  # 0 to 1
            phase = 2 * math.pi * t

            # Hip follows a simple half-sine (flexes then returns)
            hip_angle  = hip_max_deg * math.sin(phase) if phase < math.pi else 0.0

            # Knee follows with 30° phase lead and larger amplitude
            knee_phase = phase + math.radians(30)
            knee_angle = knee_max_deg * max(0.0, math.sin(knee_phase)) \
                         if knee_phase < math.pi + math.radians(30) else 0.0

            wp = JointAngles(
                ax1 = 0.0,
                ax2 = max(0.0, hip_angle),
                ax3 = max(0.0, knee_angle),
                ax4 = 0.0,
            )
            trajectory.append(self.clamp_to_safe_limits(wp))

        return trajectory

    # =========================================================================
    #  Joint Limit Validation
    # =========================================================================

    def clamp_to_safe_limits(self, angles: JointAngles) -> JointAngles:
        """
        1. Calculates the correct M4 orientation.
        2. Clamps all angles to hardware safety limits.
        """
        # FIRST: Automatically calculate Motor 4 orientation
        angles = self.fill_m4(angles)
                
        # SECOND: Apply the safety clamps from config

        j = CFG.joints
        return JointAngles(
            ax1 = max(j.hip_ab_ad.safe_min_deg,    min(j.hip_ab_ad.safe_max_deg,    angles.ax1)),
            ax2 = max(j.hip_flex_ext.safe_min_deg,  min(j.hip_flex_ext.safe_max_deg,  angles.ax2)),
            ax3 = max(j.knee_flex_ext.safe_min_deg, min(j.knee_flex_ext.safe_max_deg, angles.ax3)),
            ax4 = max(j.axis4.safe_min_deg,         min(j.axis4.safe_max_deg,         angles.ax4)),
        )


    def validate_angles(self, angles: JointAngles) -> Tuple[bool, str]:
        """
        Check if angles are within safe limits.

        Returns:
            (True, "") if valid
            (False, "reason") if any angle is out of range
        """
        j = CFG.joints
        checks = [
            (angles.ax1, j.hip_ab_ad.safe_min_deg,    j.hip_ab_ad.safe_max_deg,    "Hip Ab/Ad (ax1)"),
            (angles.ax2, j.hip_flex_ext.safe_min_deg,  j.hip_flex_ext.safe_max_deg,  "Hip Flex/Ext (ax2)"),
            (angles.ax3, j.knee_flex_ext.safe_min_deg, j.knee_flex_ext.safe_max_deg, "Knee Flex/Ext (ax3)"),
            (angles.ax4, j.axis4.safe_min_deg,          j.axis4.safe_max_deg,          "Axis 4"),
        ]
        for val, lo, hi, name in checks:
            if not (lo <= val <= hi):
                return False, f"{name}: {val:.1f}° is outside safe range [{lo}°, {hi}°]"
        return True, ""

    # =========================================================================
    #  Unit Conversion Helpers
    # =========================================================================

    def degrees_to_steps(self, degrees: float) -> int:
        """
        Convert degrees to motor steps.
        Uses the steps_per_degree from config.

        Args:
            degrees: Angle in degrees (can be negative)

        Returns:
            Number of steps (rounded to nearest integer)
        """
        return round(degrees * CFG.motors.computed_steps_per_deg)

    def steps_to_degrees(self, steps: int) -> float:
        """Convert motor steps back to degrees."""
        return steps / CFG.motors.computed_steps_per_deg

    def angles_to_steps(self, angles: JointAngles) -> List[int]:
        """
        Convert all 4 joint angles to motor steps.
        Returns [steps_ax1, steps_ax2, steps_ax3, steps_ax4]
        """
        return [
            self.degrees_to_steps(angles.ax1),
            self.degrees_to_steps(angles.ax2),
            self.degrees_to_steps(angles.ax3),
            self.degrees_to_steps(angles.ax4),
        ]

    # =========================================================================
    #  FK for Visualizer (returns dict for JSON serialization)
    # =========================================================================

    def fk_for_visualizer(self, angles: JointAngles) -> dict:
        """
        Run FK and return all joint positions as a JSON-serializable dict.
        Called by the WebSocket streamer and the 3D visualizer endpoint.

        Returns dict with:
            hip_pos, knee_pos, cuff_pos as {x, y, z} dicts
            angles as {ax1, ax2, ax3, ax4}
            L1, L2 (limb lengths for the visualizer to draw segments)
            side_sign (for left/right leg rendering)
        """
        result = self.forward_kinematics(angles)
        return {
            "hip_pos"  : {"x": round(result.hip_pos.x,  1),
                          "y": round(result.hip_pos.y,  1),
                          "z": round(result.hip_pos.z,  1)},
            "knee_pos" : {"x": round(result.knee_pos.x, 1),
                          "y": round(result.knee_pos.y, 1),
                          "z": round(result.knee_pos.z, 1)},
            "cuff_pos" : {"x": round(result.cuff_pos.x, 1),
                          "y": round(result.cuff_pos.y, 1),
                          "z": round(result.cuff_pos.z, 1)},
            "angles"   : {"ax1": round(angles.ax1, 2),
                          "ax2": round(angles.ax2, 2),
                          "ax3": round(angles.ax3, 2),
                          "ax4": round(angles.ax4, 2)},
            "L1"       : self.L1,
            "L2"       : self.L2,
            "side_sign": self.side_sign,
        }


# =============================================================================
#  Module-level singleton
#  Instantiated once when the module is imported.
#  All therapy modules use this shared instance.
# =============================================================================

_engine: Optional[KinematicsEngine] = None

def get_engine() -> KinematicsEngine:
    """Returns the shared KinematicsEngine instance, creating it if needed."""
    global _engine
    if _engine is None:
        _engine = KinematicsEngine()
    return _engine