"""Goruntu/QR akisi icin saf veri yardimcilari.

Bu dosya uav_control.py ile g_isleme_pkg arasindaki goruntu/QR mesajlarinin
duz Python veri isleriyle ilgilenir:
- vision_control payload sozluklerini olusturmak
- qr_result mesajlarini ayristirmak
- colored_field mesajlarini ayristirmak
- x_y_error mesajlarini ayristirmak

ROS tarafindaki isler uav_control.py icinde kalir:
- publishers, subscribers, timers
- std_msgs.msg.String objects
- callback wrappers
- asyncio scheduling
- log kararlari
- DroneControllerNode state guncellemeleri
- MAVSDK/offboard/action cagrilari

Buraya tasinan fonksiyonlar ihtiyac duyduklari her degeri parametre olarak alir.
Bu dosya rclpy, MAVSDK veya DroneControllerNode import etmez.
"""

import json


def build_vision_control_payload(
    is_leader,
    qr_detection_allowed,
    colored_field_detection_allowed,
    xy_error_enabled,
    target_color,
):
    payload = {
        "qr_detection_allowed": False,
        "colored_field_detection_allowed": False,
        "xy_error_enabled": False,
        "target_color": None,
    }

    if is_leader:
        payload["qr_detection_allowed"] = bool(qr_detection_allowed)

    payload["colored_field_detection_allowed"] = bool(
        colored_field_detection_allowed
    )
    payload["xy_error_enabled"] = bool(xy_error_enabled)

    if xy_error_enabled or target_color is not None:
        payload["target_color"] = target_color

    return payload


def parse_qr_result_message(message_data, fallback_timestamp_s):
    data = json.loads(message_data)

    if not isinstance(data, dict) or not bool(data.get("decoded", False)):
        return None

    payload = str(data.get("payload", "")).strip()
    if not payload:
        return None

    timestamp_s = data.get("timestamp")
    if timestamp_s is None:
        timestamp_s = fallback_timestamp_s

    return {
        "payload": payload,
        "timestamp": timestamp_s,
        "mission_plan": data.get("mission_plan"),
    }


def build_local_qr_result(payload, timestamp_s):
    return {
        "decoded": True,
        "payload": payload,
        "timestamp": float(timestamp_s),
    }


def parse_colored_field_message(message_data):
    data = json.loads(message_data)

    if not isinstance(data, dict):
        return None

    return {
        "target_type": data.get("type"),
    }


def parse_x_y_error_message(message_data):
    data = json.loads(message_data)

    if not isinstance(data, dict):
        return None

    return {
        "target_type": data.get("type"),
        "x": data.get("x", 0.0),
        "y": data.get("y", 0.0),
        "detected": data.get("detected", False),
    }
