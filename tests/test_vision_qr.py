"""Unit tests for QR and vision message helpers."""

import json

from iha_pkg import vision_qr


def test_only_leader_can_enable_qr_detection():
    follower = vision_qr.build_vision_control_payload(
        is_leader=False,
        qr_detection_allowed=True,
        colored_field_detection_allowed=False,
        xy_error_enabled=False,
        target_color=None,
    )
    leader = vision_qr.build_vision_control_payload(
        is_leader=True,
        qr_detection_allowed=True,
        colored_field_detection_allowed=False,
        xy_error_enabled=False,
        target_color=None,
    )

    assert follower["qr_detection_allowed"] is False
    assert leader["qr_detection_allowed"] is True


def test_target_color_is_preserved_for_xy_tracking():
    payload = vision_qr.build_vision_control_payload(
        is_leader=False,
        qr_detection_allowed=False,
        colored_field_detection_allowed=True,
        xy_error_enabled=True,
        target_color="red",
    )

    assert payload["colored_field_detection_allowed"] is True
    assert payload["xy_error_enabled"] is True
    assert payload["target_color"] == "red"


def test_qr_result_uses_fallback_timestamp():
    result = vision_qr.parse_qr_result_message(
        json.dumps(
            {
                "decoded": True,
                "payload": "gorev_3",
                "mission_plan": {"qr_id": 3},
            }
        ),
        fallback_timestamp_s=123.5,
    )

    assert result == {
        "payload": "gorev_3",
        "timestamp": 123.5,
        "mission_plan": {"qr_id": 3},
    }


def test_invalid_qr_result_is_rejected():
    assert (
        vision_qr.parse_qr_result_message(
            json.dumps({"decoded": False, "payload": "gorev_1"}),
            fallback_timestamp_s=1.0,
        )
        is None
    )


def test_xy_error_message_defaults_missing_values():
    result = vision_qr.parse_x_y_error_message(
        json.dumps({"type": "blue", "detected": True})
    )

    assert result == {
        "target_type": "blue",
        "x": 0.0,
        "y": 0.0,
        "detected": True,
    }
