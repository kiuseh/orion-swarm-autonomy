"""Unit tests for pure navigation helpers."""

import math

import pytest

from iha_pkg import navigation_utils as nav


def test_heading_helpers_use_the_shortest_rotation():
    assert nav.normalize_heading(-10) == pytest.approx(350)
    assert nav.get_signed_heading_difference(350, 10) == pytest.approx(20)
    assert nav.get_signed_heading_difference(10, 350) == pytest.approx(-20)
    assert nav.get_heading_difference(10, 350) == pytest.approx(20)


def test_step_towards_heading_crosses_zero_safely():
    assert nav.step_towards_heading(350, 10, 5) == pytest.approx(355)
    assert nav.step_towards_heading(8, 10, 5) == pytest.approx(10)


def test_xy_velocity_limit_preserves_direction():
    north, east = nav.limit_xy_velocity(3, 4, 2)

    assert math.hypot(north, east) == pytest.approx(2)
    assert north == pytest.approx(1.2)
    assert east == pytest.approx(1.6)


def test_coordinate_offset_and_ned_error_are_consistent():
    origin_lat = 37.412175143823063
    origin_lon = -121.998676647076721
    target_lat, target_lon = nav.add_ned_offset_to_coordinate(
        origin_lat,
        origin_lon,
        north_m=12.0,
        east_m=-7.0,
    )

    north_error, east_error = nav.global_error_to_ned_m(
        origin_lat,
        origin_lon,
        target_lat,
        target_lon,
    )

    assert north_error == pytest.approx(12.0, abs=0.01)
    assert east_error == pytest.approx(-7.0, abs=0.01)


def test_slow_radius_reduces_command_speed():
    north, east, error = nav.make_xy_velocity_from_error(
        north_error_m=1.0,
        east_error_m=0.0,
        max_xy_m_s=2.0,
        kp_xy=5.0,
        slow_radius_m=4.0,
        hold_speed_eps_m_s=0.1,
    )

    assert error == pytest.approx(1.0)
    assert math.hypot(north, east) == pytest.approx(0.5)
