"""UAV kontrolu icin saf swarm mesaj yardimcilari.

Bu dosya sadece swarm mesajlarinin duz Python veri isleriyle ilgilenir:
mesaj sozluklerini olusturur ve mesajla ilgili sozlukleri gunceller.

ROS tarafindaki isler uav_control.py icinde kalir:
- publishers, subscribers, timers
- std_msgs.msg.String objects
- callback wrappers
- asyncio scheduling
- log kararlari

Buraya tasinan fonksiyonlar ihtiyac duyduklari her degeri parametre olarak alir.
Bu dosya rclpy, MAVSDK veya DroneControllerNode import etmez.
"""


def build_swarm_info_packet(
    sender_role,
    telemetry_info,
    shared_target_reached,
    all_mission_done,
    qr_event_seq,
    qr_result,
    sender_udp_port=None,
    sender_drone_id=None,
    slot_index=None,
    state=None,
    is_follower=False,
    is_leader=False,
    takeoff_done=False,
    initial_positioning_done=False,
    rotation_alignment_done=False,
    swarm_departure_active=False,
    formation_heading_ref=None,
    rotation_active=False,
    rotating_done=False,
    leader_home_latitude=None,
    leader_home_longitude=None,
    leader_home_absolute_altitude=None,
    field_coordinate_event_seq=0,
    field_coordinates=None,
):
    # Her drone rolunun paylastigi ortak alanlar.
    packet = {
        "sender_role": sender_role,
        "latitude": telemetry_info["latitude"],
        "longitude": telemetry_info["longitude"],
        "absolute_altitude": telemetry_info["absolute_altitude"],
        "relative_altitude": telemetry_info.get("relative_altitude"),
        "heading": telemetry_info["heading"],
        "speed_m_s": telemetry_info.get("speed_m_s"),
        "battery_percent": telemetry_info.get("battery_percent"),
        "sender_udp_port": sender_udp_port,
        "sender_drone_id": sender_drone_id,
        "slot_index": slot_index,
        "state": state,
        "shared_target_reached": bool(shared_target_reached),
        "all_mission_done": bool(all_mission_done),
        "qr_event_seq": int(qr_event_seq),
        "qr_result": qr_result,
    }

    if is_follower:
        # Takipciler liderin gorev ilerlemesini takip edebilmesi icin
        # kendi durumlarini mesaja ekler.
        packet.update(
            {
                "takeoff_done": bool(takeoff_done),
                "initial_positioning_done": bool(initial_positioning_done),
                "rotation_alignment_done": bool(rotation_alignment_done),
                "swarm_departure_active": bool(swarm_departure_active),
            }
        )
    elif is_leader:
        # Lider, takipcilerin formasyon ve alan bilgilerini guncel tutar.
        packet.update(
            {
                "formation_heading_ref": formation_heading_ref,
                "rotation_active": bool(rotation_active),
                "rotating_done": bool(rotating_done),
                "leader_home_latitude": leader_home_latitude,
                "leader_home_longitude": leader_home_longitude,
                "leader_home_absolute_altitude": leader_home_absolute_altitude,
                "field_coordinate_event_seq": int(field_coordinate_event_seq),
                "field_coordinates": field_coordinates,
            }
        )

    return packet


def update_own_leader_swarm_info(
    leader_info,
    telemetry_info,
    formation_heading_ref,
    rotation_active,
    shared_target_reached,
    sender_udp_port=None,
    sender_drone_id=None,
    slot_index=None,
    state=None,
):
    leader_info["latitude"] = telemetry_info["latitude"]
    leader_info["longitude"] = telemetry_info["longitude"]
    leader_info["absolute_altitude"] = telemetry_info["absolute_altitude"]
    leader_info["relative_altitude"] = telemetry_info.get("relative_altitude")
    leader_info["heading"] = telemetry_info["heading"]
    leader_info["speed_m_s"] = telemetry_info.get("speed_m_s")
    leader_info["battery_percent"] = telemetry_info.get("battery_percent")
    leader_info["sender_udp_port"] = sender_udp_port
    leader_info["sender_drone_id"] = sender_drone_id
    leader_info["slot_index"] = slot_index
    leader_info["state"] = state
    leader_info["formation_heading_ref"] = formation_heading_ref
    leader_info["rotation_active"] = bool(rotation_active)
    leader_info["shared_target_reached"] = bool(shared_target_reached)


def update_follower_swarm_info(swarm_infos, sender_role, data):
    follower_info = swarm_infos[sender_role]
    follower_info["takeoff_done"] = data["takeoff_done"]
    follower_info["initial_positioning_done"] = data["initial_positioning_done"]
    follower_info["rotation_alignment_done"] = data["rotation_alignment_done"]
    follower_info["swarm_departure_active"] = data["swarm_departure_active"]
    follower_info["shared_target_reached"] = data["shared_target_reached"]
    follower_info["all_mission_done"] = bool(data.get("all_mission_done", False))
    follower_info["latitude"] = data.get("latitude")
    follower_info["longitude"] = data.get("longitude")
    follower_info["absolute_altitude"] = data.get("absolute_altitude")
    follower_info["relative_altitude"] = data.get("relative_altitude")
    follower_info["heading"] = data.get("heading")
    follower_info["speed_m_s"] = data.get("speed_m_s")
    follower_info["battery_percent"] = data.get("battery_percent")
    follower_info["sender_udp_port"] = data.get("sender_udp_port")
    follower_info["sender_drone_id"] = data.get("sender_drone_id")
    follower_info["slot_index"] = data.get("slot_index")
    follower_info["state"] = data.get("state")


def update_leader_swarm_basic_info(swarm_infos, data):
    leader_info = swarm_infos["lider"]
    formation_heading_ref = data.get("formation_heading_ref")
    if formation_heading_ref is None:
        leader_info["formation_heading_ref"] = None
    else:
        leader_info["formation_heading_ref"] = float(formation_heading_ref)

    leader_info["rotation_active"] = data["rotation_active"]
    leader_info["rotating_done"] = data["rotating_done"]
    leader_info["shared_target_reached"] = data["shared_target_reached"]
    leader_info["all_mission_done"] = bool(data.get("all_mission_done", False))
    leader_info["latitude"] = data["latitude"]
    leader_info["longitude"] = data["longitude"]
    leader_info["absolute_altitude"] = data["absolute_altitude"]
    leader_info["relative_altitude"] = data.get("relative_altitude")
    leader_info["heading"] = data["heading"]
    leader_info["speed_m_s"] = data.get("speed_m_s")
    leader_info["battery_percent"] = data.get("battery_percent")
    leader_info["sender_udp_port"] = data.get("sender_udp_port")
    leader_info["sender_drone_id"] = data.get("sender_drone_id")
    leader_info["slot_index"] = data.get("slot_index")
    leader_info["state"] = data.get("state")


def read_leader_home_from_swarm_info(
    data,
    current_latitude,
    current_longitude,
    current_absolute_altitude,
):
    leader_home_latitude = data.get("leader_home_latitude")
    leader_home_longitude = data.get("leader_home_longitude")
    leader_home_absolute_altitude = data.get("leader_home_absolute_altitude")

    # Konum bilgisi ancak latitude ve longitude birlikte geldiyse guncellenir.
    if leader_home_latitude is not None and leader_home_longitude is not None:
        current_latitude = float(leader_home_latitude)
        current_longitude = float(leader_home_longitude)

    # Irtifa bilgisi tek basina guncellenebilir.
    if leader_home_absolute_altitude is not None:
        current_absolute_altitude = float(leader_home_absolute_altitude)

    return current_latitude, current_longitude, current_absolute_altitude


def update_field_coordinates_from_leader(
    field_coordinates_state,
    data,
    last_event_seq,
):
    field_coordinates = data.get("field_coordinates")
    field_coordinate_event_seq = int(data.get("field_coordinate_event_seq", 0) or 0)
    if not (
        isinstance(field_coordinates, dict)
        and field_coordinate_event_seq > last_event_seq
    ):
        return last_event_seq, []

    # Daha yeni bir event geldiyse once veriyi guncelle, sonra hangi renklerin
    # degistigini cagirana bildir. Log basma isi uav_control.py icinde kalir.
    updated_color_names = []
    for color_name in ("red", "blue"):
        if update_field_coordinate_from_leader(
            field_coordinates_state,
            color_name,
            field_coordinates,
        ):
            updated_color_names.append(color_name)

    return field_coordinate_event_seq, updated_color_names


def update_field_coordinate_from_leader(
    field_coordinates_state,
    color_name,
    field_coordinates,
):
    coordinate_info = field_coordinates.get(color_name)
    if not isinstance(coordinate_info, dict):
        return False

    latitude = coordinate_info.get("latitude")
    longitude = coordinate_info.get("longitude")
    updated = False
    if latitude is not None:
        field_coordinates_state[color_name]["latitude"] = float(latitude)
        updated = True
    if longitude is not None:
        field_coordinates_state[color_name]["longitude"] = float(longitude)
        updated = True

    return updated
