"""Small settings containers for the drone controller."""

from dataclasses import dataclass


@dataclass
class FormationSettings:
    follow_distance_m: float = 5.0


@dataclass
class RotationSettings:
    heading_tolerance_deg: float = 1.0
    leader_rate_deg_s: float = 30.0
    command_period_s: float = 0.25


@dataclass
class CollisionSettings:
    trigger_distance_m: float = 2.0
    release_distance_m: float = 2.3
    avoidance_altitude_offset_m: float = 2.5
    altitude_release_slot_distance_m: float = 1.5


@dataclass
class LandingSettings:
    stabilization_wait_s: float = 2.0
    final_transition_altitude_m: float = 1.2
    blind_transition_altitude_m: float = 2.0
    final_alignment_px: float = 160.0
    final_alignment_memory_s: float = 1.0
    area_approach_relative_altitude_m: float = 5.0


@dataclass
class NavigationSettings:
    bearing_offset_deg: float = 0.0
    arrival_radius_m: float = 0.8
    final_arrival_radius_m: float = 1.0
    slow_radius_m: float = 4.0
    alt_tolerance_m: float = 0.25
    hold_speed_epsilon_m_s: float = 0.25
    kp_xy: float = 0.45
    kp_z: float = 0.80
    leader_xy_max_m_s: float = 2.0
    follower_xy_max_m_s: float = 2.4
    follower_feed_forward_gain: float = 1.0
    leader_state_latency_comp_s: float = 0.25
    initial_positioning_xy_max_m_s: float = 2.5
    rotation_xy_max_m_s: float = 2.0
    area_xy_max_m_s: float = 2.0
    return_xy_max_m_s: float = 2.0
    vertical_max_m_s: float = 0.6
    xy_accel_limit_m_s2: float = 1.2
    vertical_accel_limit_m_s2: float = 0.5
    command_period_s: float = 0.1
    offboard_prestream_count: int = 12


__all__ = [
    "CollisionSettings",
    "FormationSettings",
    "LandingSettings",
    "NavigationSettings",
    "RotationSettings",
]
