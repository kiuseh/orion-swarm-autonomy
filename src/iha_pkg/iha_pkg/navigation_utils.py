"""Pure navigation and math helpers for UAV control.

Keep this module independent from ROS, MAVSDK, and DroneControllerNode state.
Functions moved here should receive every value they need as parameters.
"""

import math

from geographiclib.geodesic import Geodesic

EARTH_RADIUS_M = 6378137.0


def get_signed_heading_difference(current, target):
    return (float(target) - float(current) + 180.0) % 360.0 - 180.0


def get_heading_difference(current, target):
    return abs(get_signed_heading_difference(current, target))


def normalize_heading(heading_deg):
    return float(heading_deg) % 360.0


def step_towards_heading(current_heading_deg, target_heading_deg, max_delta_deg):
    heading_error = get_signed_heading_difference(
        current_heading_deg,
        target_heading_deg,
    )
    if abs(heading_error) <= float(max_delta_deg):
        return normalize_heading(target_heading_deg)

    return normalize_heading(
        float(current_heading_deg) + math.copysign(float(max_delta_deg), heading_error)
    )


def calculate_bearing(lat1, lon1, lat2, lon2, bearing_offset_deg=0.0):
    result = Geodesic.WGS84.Inverse(
        float(lat1),
        float(lon1),
        float(lat2),
        float(lon2),
    )
    return normalize_heading(result["azi1"] + float(bearing_offset_deg))


def offset_coordinate(origin_lat, origin_lon, heading_deg, relative_angle_deg, distance_m):
    absolute_angle_rad = math.radians(float(heading_deg) + float(relative_angle_deg))
    origin_lat_rad = math.radians(float(origin_lat))

    delta_north = float(distance_m) * math.cos(absolute_angle_rad)
    delta_east = float(distance_m) * math.sin(absolute_angle_rad)

    delta_lat = delta_north / EARTH_RADIUS_M * (180.0 / math.pi)
    delta_lon = delta_east / (EARTH_RADIUS_M * math.cos(origin_lat_rad)) * (
        180.0 / math.pi
    )

    return float(origin_lat) + delta_lat, float(origin_lon) + delta_lon


def global_error_to_ned_m(current_lat, current_lon, target_lat, target_lon):
    mean_lat = math.radians((float(current_lat) + float(target_lat)) / 2.0)
    delta_lat = math.radians(float(target_lat) - float(current_lat))
    delta_lon = math.radians(float(target_lon) - float(current_lon))

    north_error_m = delta_lat * EARTH_RADIUS_M
    east_error_m = delta_lon * EARTH_RADIUS_M * math.cos(mean_lat)
    return north_error_m, east_error_m


def add_ned_offset_to_coordinate(origin_lat, origin_lon, north_m, east_m):
    origin_lat = float(origin_lat)
    origin_lon = float(origin_lon)
    origin_lat_rad = math.radians(origin_lat)

    delta_lat = float(north_m) / EARTH_RADIUS_M * (180.0 / math.pi)
    delta_lon = (
        float(east_m) / (EARTH_RADIUS_M * math.cos(origin_lat_rad)) * (180.0 / math.pi)
    )
    return origin_lat + delta_lat, origin_lon + delta_lon


def distance_to_target_m(lat1, lon1, lat2, lon2):
    result = Geodesic.WGS84.Inverse(
        float(lat1),
        float(lon1),
        float(lat2),
        float(lon2),
    )
    return float(result["s12"])


def limit_xy_velocity(north_m_s, east_m_s, max_xy_m_s):
    max_xy_m_s = max(0.0, float(max_xy_m_s))
    speed_m_s = math.hypot(float(north_m_s), float(east_m_s))
    if speed_m_s <= max_xy_m_s or speed_m_s == 0.0:
        return float(north_m_s), float(east_m_s)

    scale = max_xy_m_s / speed_m_s
    return float(north_m_s) * scale, float(east_m_s) * scale


def make_xy_velocity_from_error(
    north_error_m,
    east_error_m,
    max_xy_m_s,
    kp_xy,
    slow_radius_m,
    hold_speed_eps_m_s,
):
    horizontal_error_m = math.hypot(float(north_error_m), float(east_error_m))
    north_m_s = float(kp_xy) * float(north_error_m)
    east_m_s = float(kp_xy) * float(east_error_m)

    effective_max_xy_m_s = float(max_xy_m_s)
    if 0.0 < horizontal_error_m < float(slow_radius_m):
        effective_max_xy_m_s = max(
            float(hold_speed_eps_m_s),
            float(max_xy_m_s) * horizontal_error_m / float(slow_radius_m),
        )

    north_m_s, east_m_s = limit_xy_velocity(
        north_m_s,
        east_m_s,
        effective_max_xy_m_s,
    )
    return north_m_s, east_m_s, horizontal_error_m


def make_xy_velocity_from_global_points(
    current_lat,
    current_lon,
    target_lat,
    target_lon,
    max_xy_m_s,
    kp_xy,
    slow_radius_m,
    hold_speed_eps_m_s,
):
    north_error_m, east_error_m = global_error_to_ned_m(
        current_lat,
        current_lon,
        target_lat,
        target_lon,
    )
    return make_xy_velocity_from_error(
        north_error_m,
        east_error_m,
        max_xy_m_s,
        kp_xy,
        slow_radius_m,
        hold_speed_eps_m_s,
    )
