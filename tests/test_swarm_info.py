"""Unit tests for swarm message helpers."""

from iha_pkg import swarm_info


TELEMETRY = {
    "latitude": 37.4,
    "longitude": -121.9,
    "absolute_altitude": 20.0,
    "relative_altitude": 8.0,
    "heading": 90.0,
    "speed_m_s": 2.5,
    "battery_percent": 81.0,
}


def test_follower_packet_contains_follower_flags():
    packet = swarm_info.build_swarm_info_packet(
        sender_role="sol_1",
        telemetry_info=TELEMETRY,
        shared_target_reached=False,
        all_mission_done=False,
        qr_event_seq=0,
        qr_result=None,
        sender_udp_port=14541,
        sender_drone_id=1,
        slot_index=-1,
        state="INITIAL_POSITIONING",
        is_follower=True,
        takeoff_done=True,
        initial_positioning_done=False,
        rotation_alignment_done=False,
        swarm_departure_active=False,
    )

    assert packet["sender_role"] == "sol_1"
    assert packet["takeoff_done"] is True
    assert packet["initial_positioning_done"] is False
    assert "formation_heading_ref" not in packet


def test_leader_packet_contains_shared_reference():
    packet = swarm_info.build_swarm_info_packet(
        sender_role="lider",
        telemetry_info=TELEMETRY,
        shared_target_reached=True,
        all_mission_done=False,
        qr_event_seq=2,
        qr_result={"payload": "gorev_2"},
        is_leader=True,
        formation_heading_ref=135.0,
        rotation_active=True,
        rotating_done=False,
        field_coordinate_event_seq=3,
        field_coordinates={"red": {"latitude": 37.5, "longitude": -121.8}},
    )

    assert packet["formation_heading_ref"] == 135.0
    assert packet["rotation_active"] is True
    assert packet["field_coordinate_event_seq"] == 3
    assert "takeoff_done" not in packet


def test_field_coordinates_update_only_for_a_new_event():
    state = {
        "red": {"latitude": None, "longitude": None},
        "blue": {"latitude": None, "longitude": None},
    }
    message = {
        "field_coordinate_event_seq": 2,
        "field_coordinates": {
            "red": {"latitude": 37.51, "longitude": -121.81},
        },
    }

    event_seq, updated = swarm_info.update_field_coordinates_from_leader(
        state,
        message,
        last_event_seq=1,
    )
    repeated_seq, repeated = swarm_info.update_field_coordinates_from_leader(
        state,
        message,
        last_event_seq=event_seq,
    )

    assert event_seq == 2
    assert updated == ["red"]
    assert state["red"] == {"latitude": 37.51, "longitude": -121.81}
    assert repeated_seq == 2
    assert repeated == []


def test_leader_home_requires_a_complete_coordinate_pair():
    latitude, longitude, altitude = swarm_info.read_leader_home_from_swarm_info(
        {
            "leader_home_latitude": 37.7,
            "leader_home_absolute_altitude": 42.0,
        },
        current_latitude=37.4,
        current_longitude=-121.9,
        current_absolute_altitude=20.0,
    )

    assert latitude == 37.4
    assert longitude == -121.9
    assert altitude == 42.0
