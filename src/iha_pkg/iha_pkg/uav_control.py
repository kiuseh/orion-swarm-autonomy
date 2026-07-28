import asyncio
import json
import math
import threading
import time

import rclpy
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, VelocityNedYaw
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from iha_pkg.controller_settings import (
    CollisionSettings,
    FormationSettings,
    LandingSettings,
    NavigationSettings,
    RotationSettings,
)
from iha_pkg.mission_types import DroneRole, DroneState
from iha_pkg import navigation_utils as nav
from iha_pkg import swarm_info
from iha_pkg import vision_qr

FORMATION_ANGLES = {
    "Okbaşı": {"sol": -150.0, "sag": 150.0},
    "V": {"sol": -30.0, "sag": 30.0},
    "Çizgi": {"sol": -90.0, "sag": 90.0},
}

PITCH_ALTITUDE_FACTORS = {
    "V": {"sol": 1.0, "lider": -1.0, "sag": 1.0},
    "Okbaşı": {"sol": -1.0, "lider": 1.0, "sag": -1.0},
}

ROLL_ALTITUDE_FACTORS = {
    "sol": 1.0,
    "lider": 0.0,
    "sag": -1.0,
}

DEFAULT_FORMATION_TYPE = "Çizgi"

ARAYUZ_TOPIC = "arayuz_topic"
SWARM_INFO_TOPIC = "swarm_info_topic"
NEW_MISSION_TOPIC = "new_mission_topic"
EMERGENCY_LAND_TOPIC = "emergency_land_topic"
VISION_CONTROL_TOPIC = "vision_control_topic"
COLORED_FIELD_TOPIC = "colored_field_topic"
X_Y_ERROR_TOPIC = "x_y_error_topic"
QR_RESULT_TOPIC = "qr_result_topic"
BASE_UDP_PORT = 14540
LEADER_ROLE_KEY = "lider"
OWN_HOME_WAIT_MESSAGE = "drone'un kendi home koordinatı bekleniyor, eve dönüş ertelendi."
RETURN_HOME_STARTED_EVENT_KEY = "eve dönüş başladı."
RETURN_HOME_ARRIVAL_RADIUS_M = 1.5


def udp_port_to_drone_id(udp_port):
    return int(udp_port) - BASE_UDP_PORT


def slot_index_to_role_key(slot_index):
    slot_index = int(slot_index)
    if slot_index == 0:
        return LEADER_ROLE_KEY
    if slot_index < 0:
        return f"sol_{abs(slot_index)}"
    return f"sag_{slot_index}"


def slot_index_to_side_key(slot_index):
    slot_index = int(slot_index)
    if slot_index < 0:
        return "sol"
    if slot_index > 0:
        return "sag"
    return LEADER_ROLE_KEY


class DroneControllerNode(Node):
    def __init__(self):
        super().__init__("drone_controller_node")
        self.log = self.get_logger()

        # ROS parametreleri.
        self.declare_parameter("udp_port", 14540)
        self.declare_parameter("mavsdk_port", 50051)
        self.declare_parameter("bearing_offset_deg", 0.0)
        self.declare_parameter("rotation_heading_tolerance_deg", 1.0)

        # Parametrelerden okunan bağlantı ayarları.
        self.own_udp_port_ = (
            self.get_parameter("udp_port").get_parameter_value().integer_value
        )
        self.own_drone_id_ = udp_port_to_drone_id(self.own_udp_port_)
        self.own_mavsdk_port_ = (
            self.get_parameter("mavsdk_port").get_parameter_value().integer_value
        )
        self.log = self.get_logger().get_child(str(self.own_udp_port_))
        self.vision_control_topic_ = f"udp_{self.own_udp_port_}_{VISION_CONTROL_TOPIC}"
        self.colored_field_topic_ = f"udp_{self.own_udp_port_}_{COLORED_FIELD_TOPIC}"
        self.x_y_error_topic_ = f"udp_{self.own_udp_port_}_{X_Y_ERROR_TOPIC}"
        self.qr_result_topic_ = f"udp_{self.own_udp_port_}_{QR_RESULT_TOPIC}"

        # Drone ve rol durumu.
        self.drone = System(port=self.own_mavsdk_port_)
        self.state_ = DroneState.IDLE
        self.role_ = None
        self.role_str_ = None
        self.own_slot_index_ = None

        # Async event loop altyapısı.
        self.loop = asyncio.new_event_loop()
        self.async_thread_ = threading.Thread(target=self._run_event_loop, daemon=True)
        self.async_thread_.start()

        # Görev, formasyon ve irtifa runtime bağlamı.
        self.formation_type_ = None
        self.is_swarm_departure_active = False
        self.target_color_ = None
        self.relative_angle_ = None
        self.swarm_center_altitude_ = None
        self.mission_altitude_ = None
        self.original_mission_altitude_ = None
        self.maneuver_pitch_deg_ = 0.0
        self.maneuver_roll_deg_ = 0.0
        self.collision_control_mission_altitude_ = None
        self.home_absolute_altitude_ = None
        self.home_latitude_ = None
        self.home_longitude_ = None
        self.takeoff_altitude_ = None
        self.rotation_target_heading_ = None
        self.formation_heading_ref_ = None
        self.rotation_active_ = False
        self.rotation_alignment_done_ = False
        self.leader_altitude_prepared_ = False

        # Görev ilerleme bayrakları.
        self.is_arm_mission_done_ = False
        self.is_takeoff_mission_done_ = False
        self.is_rotating_mission_done_ = False
        self.is_initial_positioning_mission_done_ = False
        self.is_all_mission_done_ = False
        self.gcs_mission_plan_ = None
        self.is_emergency_land_active = False
        self.reset_return_home_state()

        # Güvenlik runtime durumu.
        self.is_collision_control_safe_ = True

        # Sürü paylaşım verileri.
        self.swarm_infos_ = {}
        self.target_positions_ = {}
        self.role_slot_indices_ = {}
        self.role_key_by_udp_port_ = {}
        self.role_key_by_drone_id_ = {}
        self.field_coordinates_ = {
            "red": {
                "latitude": 37.412218122205,
                "longitude": -121.998559622969,
            },
            "blue": {"latitude": None, "longitude": None},
        }
        self.x_y_error_ = {
            "x": 0.0,
            "y": 0.0,
            "detected": False,
            "type": None,
        }

        # Görüntü işleme ve QR koordinasyonu.
        self.qr_detection_allowed_ = False
        self.local_qr_event_seq_ = 0
        self.local_qr_result_ = None
        self.field_coordinate_event_seq_ = 0
        self.last_leader_field_coordinate_event_seq_ = 0
        self.leader_home_latitude_ = None
        self.leader_home_longitude_ = None
        self.leader_home_absolute_altitude_ = None

        # Telemetri ve log durumu.
        self.telemetry_info_ = {
            "latitude": None,
            "longitude": None,
            "absolute_altitude": None,
            "relative_altitude": None,
            "heading": None,
            "speed_m_s": None,
            "battery_percent": None,
        }
        self.is_telemetry_collection_started_ = False
        self.logged_events_ = set()

        # Formasyon ayarları.
        self.formation_settings_ = FormationSettings()

        # Rotasyon ayarları.
        self.rotation_settings_ = RotationSettings()

        # Çarpışma önleme ayarları.
        self.collision_settings_ = CollisionSettings()

        # İniş ve alan yaklaşma ayarları.
        self.landing_settings_ = LandingSettings()

        # Velocity-NED navigasyon ve heading ayarları.
        self.nav_settings_ = NavigationSettings()

        # Bağlantı ve temel komut gönderim bayrakları.
        self.is_connected_ = False
        self.takeoff_command_sent_ = False
        self.arm_command_sent_ = False
        self.is_initial_positioning_command_sent_ = False
        self.move_command_sent_ = False
        self.land_command_sent_ = False
        self.return_home_land_command_sent_ = False

        # Ayar olmayan, uçuş sırasında güncellenen oturum durumu.
        self.rotation_last_update_ = 0.0
        self.nav_offboard_active_ = False
        self.last_ned_velocity_cmd_ = (0.0, 0.0, 0.0)
        self.last_ned_velocity_cmd_time_s_ = None
        self.last_xy_alignment_time_ = 0.0
        self.land_to_area_entry_time_ = 0.0
        self.land_to_area_stabilized_ = False
        self.landing_alignment_locked_ = False
        self.is_collision_altitude_command_sent_ = False
        self.is_departure_navigation_command_sent_ = False
        self.is_rotation_command_sent_ = False
        self.is_area_location_reached_ = False
        self.is_landed_ = False
        self.leader_altitude_prepare_command_sent_ = False

        # Callback çalışma kilitleri.
        self.lider_kontrol_busy_ = False
        self.leader_info_pub_busy_ = False
        self.takipci_kontrol_busy_ = False

        # ROS callback grupları.
        self.lider_kontrol_grup_ = ReentrantCallbackGroup()
        self.takipci_kontrol_grubu_ = ReentrantCallbackGroup()
        self.dinleyici_grubu_ = ReentrantCallbackGroup()
        self.swarm_info_group_ = ReentrantCallbackGroup()

        # ROS publisher/subscriber/timer handle'ları.
        self.leader_controller_timer_ = None
        self.follower_controller_timer_ = None
        self.publish_swarm_info_timer_ = None
        self.vision_control_timer_ = None
        self.swarm_info_publisher_ = None
        self.vision_control_publisher_ = None
        self.new_mission_publisher_ = None
        self.swarm_info_subscriber_ = None
        self.new_mission_subscriber_ = None
        self.colored_field_subscriber_ = None
        self.emergency_land_subscriber_ = None
        self.x_y_error_subscriber_ = None
        self.qr_result_subscriber_ = None
        self.is_pub_sub_created_ = False
        self.qr_wait_command_sent_ = False
        self.qr_wait_delay_deadline_s_ = None

        # İlk komut aboneliği.
        self.command_subscriber_ = self.create_subscription(
            String,
            ARAYUZ_TOPIC,
            self.command_callback,
            10,
            callback_group=self.dinleyici_grubu_,
        )
        self.emergency_land_subscriber_ = self.create_subscription(
            String,
            EMERGENCY_LAND_TOPIC,
            self.emergency_land_callback_wrapper,
            10,
            callback_group=self.dinleyici_grubu_,
        )

        # Arka plan bağlantı görevi.
        asyncio.run_coroutine_threadsafe(self.connect_drone(), self.loop)

        # Başlangıç logu.
        self.log.info(
            f"{self.own_udp_port_} Drone Controller Node başlatıldı.\n"
            f"udp portu: {self.own_udp_port_}\n"
            f"drone id: {self.own_drone_id_}\n"
            f"MAVSDK gRPC portu: {self.own_mavsdk_port_}\n"
            f"vision topic: {self.vision_control_topic_}\n"
            f"colored field topic: {self.colored_field_topic_}\n"
            f"x/y error topic: {self.x_y_error_topic_}\n"
            f"qr result topic: {self.qr_result_topic_}\n"
        )

    def parse_role_slots_from_plan(self, mission_plan):
        raw_roles = mission_plan.get("roles")
        if not raw_roles:
            raise ValueError("roles alani boş")

        role_slots = {}
        used_slots = {}
        for raw_udp_port, raw_slot_data in raw_roles.items():
            udp_port = int(raw_udp_port)
            slot_index = int(raw_slot_data)
            if slot_index in used_slots:
                raise ValueError(
                    f"slot_index tekrari: {slot_index} "
                    f"({used_slots[slot_index]} ve {udp_port})"
                )

            role_slots[udp_port] = slot_index
            used_slots[slot_index] = udp_port

        leader_count = sum(1 for slot_index in role_slots.values() if slot_index == 0)
        if leader_count != 1:
            raise ValueError("roles icinde tam olarak bir lider slotu (0) olmali")
        if self.own_udp_port_ not in role_slots:
            raise ValueError(f"bu drone'un udp portu roles icinde yok: {self.own_udp_port_}")

        return role_slots

    def configure_roles_from_message(self, role_config):
        if self.gcs_mission_plan_ is not None:
            self.log.warning(
                "Görev başladıktan sonra gelen rol konfigürasyonu yok sayıldı."
            )
            return

        try:
            role_slots = self.parse_role_slots_from_plan(role_config)
            self.initialize_swarm_state_from_slots(role_slots)
            self.configure_own_role_from_slot(role_slots[self.own_udp_port_])
        except Exception as e:
            self.log.error(f"Rol konfigürasyonu uygulanamadı: {e}")
            return

        if not self.is_pub_sub_created_:
            self.create_mission_pub_sub()

        self.log.info(
            "Rol konfigürasyonu uygulandı: "
            f"role={self.role_str_}, slot_index={self.own_slot_index_}"
        )

    def make_empty_swarm_info(self):
        return {
            "takeoff_done": False,
            "swarm_departure_active": False,
            "initial_positioning_done": False,
            "rotation_alignment_done": False,
            "shared_target_reached": False,
            "all_mission_done": False,
            "latitude": None,
            "longitude": None,
            "absolute_altitude": None,
            "relative_altitude": None,
            "heading": None,
            "speed_m_s": None,
            "battery_percent": None,
            "sender_udp_port": None,
            "sender_drone_id": None,
            "slot_index": None,
            "state": None,
            "formation_heading_ref": None,
            "rotation_active": False,
            "rotating_done": False,
        }

    def initialize_swarm_state_from_slots(self, role_slots):
        self.swarm_infos_ = {}
        self.target_positions_ = {}
        self.role_slot_indices_ = {}
        self.role_key_by_udp_port_ = {}
        self.role_key_by_drone_id_ = {}

        for udp_port, slot_index in sorted(role_slots.items()):
            role_key = slot_index_to_role_key(slot_index)
            self.role_slot_indices_[role_key] = int(slot_index)
            self.role_key_by_udp_port_[int(udp_port)] = role_key
            self.role_key_by_drone_id_[udp_port_to_drone_id(udp_port)] = role_key
            self.swarm_infos_[role_key] = self.make_empty_swarm_info()
            self.swarm_infos_[role_key]["sender_udp_port"] = int(udp_port)
            self.swarm_infos_[role_key]["sender_drone_id"] = udp_port_to_drone_id(
                udp_port
            )
            self.swarm_infos_[role_key]["slot_index"] = int(slot_index)
            self.target_positions_[role_key] = {
                "latitude": None,
                "longitude": None,
            }

    def configure_own_role_from_slot(self, slot_index):
        self.own_slot_index_ = int(slot_index)
        self.role_str_ = slot_index_to_role_key(self.own_slot_index_)

        if self.own_slot_index_ == 0:
            self.role_ = DroneRole.LEADER
            self.relative_angle_ = None
        elif self.own_slot_index_ < 0:
            self.role_ = DroneRole.SOL
            self.relative_angle_ = self.get_role_relative_angle(self.role_str_)
        else:
            self.role_ = DroneRole.SAG
            self.relative_angle_ = self.get_role_relative_angle(self.role_str_)

    def is_leader_role(self, role_name=None):
        role_name = role_name or self.role_str_
        return role_name == LEADER_ROLE_KEY

    def is_follower_role(self, role_name=None):
        role_name = role_name or self.role_str_
        return role_name in self.role_slot_indices_ and role_name != LEADER_ROLE_KEY

    def get_role_slot_index(self, role_name=None):
        role_name = role_name or self.role_str_
        return self.role_slot_indices_.get(role_name)

    def get_role_side_key(self, role_name=None):
        slot_index = self.get_role_slot_index(role_name)
        if slot_index is None:
            return None
        return slot_index_to_side_key(slot_index)

    def get_role_slot_distance_scale(self, role_name=None):
        slot_index = self.get_role_slot_index(role_name)
        if slot_index is None:
            return 1.0
        return max(1.0, float(abs(int(slot_index))))

    def get_follower_role_keys(self):
        return [
            role_name
            for role_name, slot_index in sorted(
                self.role_slot_indices_.items(),
                key=lambda item: (abs(int(item[1])), int(item[1])),
            )
            if int(slot_index) != 0
        ]

    def get_other_follower_role_keys(self):
        return [
            role_name
            for role_name in self.get_follower_role_keys()
            if role_name != self.role_str_
        ]

    def are_all_followers_flag_true(self, flag_name):
        return all(
            bool(self.swarm_infos_[role_name].get(flag_name, False))
            for role_name in self.get_follower_role_keys()
        )

    def command_callback(self, msg):
        if self.loop is None:
            return

        # Görev planını arayüz mesajından oku. Görev planı olmayan YKİ mesajları
        # kontrol state'ini bozmasın diye burada elenir.
        try:
            mission_plan = json.loads(msg.data)
        except Exception as e:
            self.log.error(f"gcs mission plan parse hatası: {e}")
            return

        if not isinstance(mission_plan, dict):
            self.log.error("gcs mission plan dict formatında olmalı")
            return

        if mission_plan.get("command") == "configure_roles":
            self.configure_roles_from_message(mission_plan)
            return

        required_plan_fields = {
            "roles",
            "takeoff_altitude",
            "follow_distance_m",
            "latitude",
            "longitude",
        }
        missing_plan_fields = required_plan_fields.difference(mission_plan)
        if missing_plan_fields:
            self.log.warning(
                "görev planı olmayan arayüz mesajı yok sayıldı: "
                f"eksik alanlar={sorted(missing_plan_fields)}"
            )
            return

        # Bu drone'a atanan slotu belirle ve suru durum sozluklerini plana gore kur.
        try:
            role_slots = self.parse_role_slots_from_plan(mission_plan)
            self.initialize_swarm_state_from_slots(role_slots)
            self.configure_own_role_from_slot(role_slots[self.own_udp_port_])
        except Exception as e:
            self.log.error(f"Drone rol ataması sırasında bir hata meydana geldi: {e}")
            return

        # Kalkış irtifası ve lider hedef koordinatı görev planında zorunlu alanlardır.
        try:
            if "takeoff_altitude" not in mission_plan:
                raise KeyError("takeoff_altitude")
            if "follow_distance_m" not in mission_plan:
                raise KeyError("follow_distance_m")
            if "latitude" not in mission_plan or "longitude" not in mission_plan:
                raise KeyError("latitude/longitude")

            self.takeoff_altitude_ = float(mission_plan["takeoff_altitude"])
            self.formation_settings_.follow_distance_m = float(
                mission_plan["follow_distance_m"]
            )
            if self.formation_settings_.follow_distance_m <= 0:
                raise ValueError("follow_distance_m 0'dan büyük olmalı")
            self.swarm_center_altitude_ = self.home_absolute_altitude_ + self.takeoff_altitude_
            self.mission_altitude_ = self.swarm_center_altitude_
            self.original_mission_altitude_ = self.mission_altitude_
            self.target_positions_["lider"]["latitude"] = float(mission_plan["latitude"])
            self.target_positions_["lider"]["longitude"] = float(mission_plan["longitude"])
            self.reset_return_home_state()
            self.reset_navigation_session_state()
        except Exception as e:
            self.log.error(f"plan bilgileri eksik veya hatalı: {e}")
            return

        self.gcs_mission_plan_ = mission_plan

        # Role göre ana kontrol döngüsünü başlat.
        try:
            if self.role_ == DroneRole.LEADER and self.leader_controller_timer_ is None:
                self.leader_controller_timer_ = self.create_timer(
                    0.1,
                    self.lider_kontrol_dongusu_wrapper,
                    callback_group=self.lider_kontrol_grup_,
                )
                self.log.info("lider kontrol döngüsü çalıştırıldı")
            elif self.role_ in [DroneRole.SAG, DroneRole.SOL] and self.follower_controller_timer_ is None:
                self.follower_controller_timer_ = self.create_timer(
                    0.1,
                    self.takipci_kontrol_dongusu_wrapper,
                    callback_group=self.takipci_kontrol_grubu_,
                )
                self.log.info("takipçi kontrol döngüsü çalıştırıldı")
        except Exception as e:
            self.log.error(
                f"kontrol döngüleri oluşturulurken bir hata meydana geldi: {e}"
            )

        # Görev başladıktan sonra ihtiyaç duyulan pub/sub yapıları bir kez oluşturulur.
        if not self.is_pub_sub_created_:
            self.create_mission_pub_sub()

        self.log.info("görev alındı")
        self.log.info(
            "plan:\n"
            f"role: {self.role_str_}\n"
            f"slot_index: {self.own_slot_index_}\n"
            f"target_latitude: {self.target_positions_['lider']['latitude']}\n"
            f"target_longitude: {self.target_positions_['lider']['longitude']}\n"
            f"takeoff_altitude: {self.takeoff_altitude_}\n"
            f"follow_distance_m: {self.formation_settings_.follow_distance_m}"
        )

        # Yeni görev için görev ilerleme durumunu sıfırla.
        self.is_takeoff_mission_done_ = False
        self.is_rotating_mission_done_ = False
        self.is_all_mission_done_ = False
        self.shared_target_reached_ = False
        self.local_qr_result_ = None
        self.logged_events_.clear()
        self.reset_qr_wait_state(disable_scanner=True, reason="initial_command")
        self.publish_vision_control()

    def emergency_land_callback_wrapper(self, msg):
        self.is_emergency_land_active = True
        asyncio.run_coroutine_threadsafe(self.emergency_land_callback(), self.loop)

    async def emergency_land_callback(self):
        offboard_stopped = await self.stop_ned_navigation()
        if not offboard_stopped:
            self.log.error("Acil inis oncesi offboard kapatilamadi; land komutu yine de deneniyor.")

        try:
            await self.drone.action.land()
        except ActionError as e:
            self.log.error(f"Acil inis komutu hatasi: {e._result_str}")

    def _run_event_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def reset_x_y_error_state(self):
        self.x_y_error_["x"] = 0.0
        self.x_y_error_["y"] = 0.0
        self.x_y_error_["detected"] = False
        self.x_y_error_["type"] = None

    def reset_land_to_area_session_state(self, clear_xy_error=False):
        self.land_to_area_entry_time_ = 0.0
        self.land_to_area_stabilized_ = False
        self.landing_alignment_locked_ = False
        self.last_xy_alignment_time_ = 0.0

        if clear_xy_error:
            self.reset_x_y_error_state()

    def reset_navigation_session_state(self):
        self.is_initial_positioning_command_sent_ = False
        self.is_collision_altitude_command_sent_ = False
        self.collision_control_mission_altitude_ = None
        self.move_command_sent_ = False
        self.is_rotation_command_sent_ = False
        self.rotation_last_update_ = 0.0
        self.rotation_target_heading_ = None
        self.formation_heading_ref_ = None
        self.rotation_active_ = False
        self.rotation_alignment_done_ = False
        self.leader_altitude_prepared_ = False
        self.leader_altitude_prepare_command_sent_ = False
        self.nav_offboard_active_ = False
        self.reset_ned_velocity_limiter()

    def reset_return_home_state(self):
        self.is_return_to_home_after_mission_ = False
        self.return_home_after_current_mission_ = False
        self.shared_target_reached_ = False
        self.return_home_land_command_sent_ = False

    def reset_return_home_sequence(self):
        self.is_return_to_home_after_mission_ = False
        self.return_home_land_command_sent_ = False
        self.clear_logged_events(
            OWN_HOME_WAIT_MESSAGE,
            RETURN_HOME_STARTED_EVENT_KEY,
        )

    def log_once(self, message, event_key=None):
        if event_key is None:
            event_key = message
        if event_key in self.logged_events_:
            return
        self.log.info(message)
        self.logged_events_.add(event_key)

    def clear_logged_events(self, *event_keys):
        for event_key in event_keys:
            self.logged_events_.discard(event_key)

    def create_mission_pub_sub(self):
        try:
            self.swarm_info_publisher_ = self.create_publisher(String, SWARM_INFO_TOPIC, 10)
            self.vision_control_publisher_ = self.create_publisher(
                String,
                self.vision_control_topic_,
                10,
            )
            self.new_mission_publisher_ = self.create_publisher(
                String,
                NEW_MISSION_TOPIC,
                10,
            )
            self.publish_swarm_info_timer_ = self.create_timer(
                0.1,
                self.publish_swarm_info_wrapper,
                callback_group=self.swarm_info_group_,
            )
            self.vision_control_timer_ = self.create_timer(
                1.0,
                self.publish_vision_control,
                callback_group=self.dinleyici_grubu_,
            )

            self.swarm_info_subscriber_ = self.create_subscription(
                String,
                SWARM_INFO_TOPIC,
                self.swarm_info_subscriber_callback,
                10,
            )
            self.new_mission_subscriber_ = self.create_subscription(
                String,
                NEW_MISSION_TOPIC,
                self.new_mission_subscriber_callback_wrapper,
                10,
            )
            if self.emergency_land_subscriber_ is None:
                self.emergency_land_subscriber_ = self.create_subscription(
                    String,
                    EMERGENCY_LAND_TOPIC,
                    self.emergency_land_callback_wrapper,
                    10,
                )
            self.colored_field_subscriber_ = self.create_subscription(
                String,
                self.colored_field_topic_,
                self.colored_field_subscriber_callback,
                10,
            )
            self.x_y_error_subscriber_ = self.create_subscription(
                String,
                self.x_y_error_topic_,
                self.x_y_error_callback,
                10,
            )
            self.qr_result_subscriber_ = self.create_subscription(
                String,
                self.qr_result_topic_,
                self.qr_result_callback,
                10,
            )

            self.is_pub_sub_created_ = True
        except Exception as e:
            self.log.error(
                f"publisher ve subscriber yapıları oluşturulurken bir hata meydana geldi: {e}"
            )
            self.is_pub_sub_created_ = False

    def publish_vision_control(self):
        if self.vision_control_publisher_ is None:
            return

        payload = vision_qr.build_vision_control_payload(
            is_leader=self.role_ == DroneRole.LEADER,
            qr_detection_allowed=self.qr_detection_allowed_,
            colored_field_detection_allowed=(
                self.is_leader_colored_field_detection_active()
            ),
            xy_error_enabled=self.is_xy_error_tracking_active(),
            target_color=self.target_color_,
        )

        msg = String()
        msg.data = json.dumps(payload)
        self.vision_control_publisher_.publish(msg)

    def is_leader_colored_field_detection_active(self):
        return (
            self.role_ == DroneRole.LEADER
            and self.state_ == DroneState.MOVE
            and not self.shared_target_reached_
            and not self.is_all_mission_done_
            and not self.is_return_to_home_after_mission_
        )

    def is_xy_error_tracking_active(self):
        return (
            self.role_ in (DroneRole.SOL, DroneRole.SAG)
            and self.is_swarm_departure_active
            and self.target_color_ is not None
            and self.is_area_location_reached_
            and not self.is_landed_
        )

    def set_qr_detection_allowed(self, enabled, reason="", force_publish=False):
        enabled = bool(enabled)
        changed = self.qr_detection_allowed_ != enabled
        self.qr_detection_allowed_ = enabled
        if changed:
            self.log.info(
                f"Lider QR izni {'acildi' if enabled else 'kapatildi'}."
                f" reason={reason}"
            )
        if changed or force_publish:
            self.publish_vision_control()

    def reset_qr_wait_state(self, disable_scanner=False, reason="reset"):
        if disable_scanner:
            self.set_qr_detection_allowed(False, reason, force_publish=True)
        self.qr_wait_command_sent_ = False
        self.qr_wait_delay_deadline_s_ = None
        self.clear_logged_events("qr_wait_delay")

    def update_leader_qr_wait_state(self):
        if self.role_ != DroneRole.LEADER or self.vision_control_publisher_ is None:
            return

        if self.return_home_after_current_mission_ or self.is_return_to_home_after_mission_:
            self.reset_qr_wait_state(disable_scanner=True, reason="return_home")
            return

        all_roles_completed_current_mission = bool(
            self.is_all_mission_done_
            and self.are_all_followers_flag_true("all_mission_done")
        )
        if not all_roles_completed_current_mission:
            self.set_qr_detection_allowed(False, "mission_incomplete")
            self.qr_wait_delay_deadline_s_ = None
            self.clear_logged_events("qr_wait_delay")
            return

        if self.qr_wait_command_sent_:
            return

        if self.qr_wait_delay_deadline_s_ is None:
            self.qr_wait_delay_deadline_s_ = time.monotonic() + 2.0
            self.log_once(
                "tum drone'lar gorevi tamamladi, 2 saniye sonra lider QR dinlemeye gececek.",
                "qr_wait_delay",
            )
            return

        if time.monotonic() < self.qr_wait_delay_deadline_s_:
            return

        self.set_qr_detection_allowed(True, "mission_complete")
        self.qr_wait_command_sent_ = True
        self.qr_wait_delay_deadline_s_ = None
        self.clear_logged_events("qr_wait_delay")
        self.log.info("Lider QR dinlemeye gecti.")

    def qr_result_callback(self, msg):
        try:
            qr_message = vision_qr.parse_qr_result_message(
                msg.data,
                fallback_timestamp_s=time.time(),
            )
        except Exception as e:
            self.log.error(f"qr_result mesajı okunamadı: {e}")
            return

        if qr_message is None:
            return

        payload = qr_message["payload"]
        timestamp_s = qr_message["timestamp"]
        mission_plan = qr_message["mission_plan"]

        self.local_qr_result_ = vision_qr.build_local_qr_result(payload, timestamp_s)
        self.local_qr_event_seq_ += 1
        self.log.info(f"Yerel QR payload alındı: '{payload}'")

        if self.role_ != DroneRole.LEADER:
            return

        self.reset_qr_wait_state(disable_scanner=True, reason="qr_decoded")
        if self.new_mission_publisher_ is None:
            self.log.error("new_mission publisher henuz hazir degil.")
            return

        new_mission_msg = String()
        new_mission_msg.data = json.dumps(mission_plan)
        self.new_mission_publisher_.publish(new_mission_msg)
        self.log.info(f"QR gorevi yayinlandi: payload='{payload}'")

    async def start_telemetry_collection(self):
        if self.is_telemetry_collection_started_:
            return

        try:
            await self.drone.telemetry.set_rate_position(20.0)
            set_rate_velocity_ned = getattr(
                self.drone.telemetry,
                "set_rate_velocity_ned",
                None,
            )
            if set_rate_velocity_ned is not None:
                await set_rate_velocity_ned(20.0)

            asyncio.create_task(self.collect_position_stream())
            asyncio.create_task(self.collect_heading_stream())
            asyncio.create_task(self.collect_velocity_stream())
            asyncio.create_task(self.collect_battery_stream())
            self.is_telemetry_collection_started_ = True
        except Exception as e:
            self.log.error(
                f"collector fonksiyonlar başlatılırken hata meydana geldi! Hata: {e}"
            )

    async def collect_position_stream(self):
        while True:
            try:
                async for position in self.drone.telemetry.position():
                    self.telemetry_info_["latitude"] = position.latitude_deg
                    self.telemetry_info_["longitude"] = position.longitude_deg
                    self.telemetry_info_["absolute_altitude"] = position.absolute_altitude_m
                    self.telemetry_info_["relative_altitude"] = position.relative_altitude_m
            except Exception as e:
                self.log.error(f"position stream koptu! 0.2 saniye bekleniyor... Hata: {e}")
                await asyncio.sleep(0.2)

    async def collect_heading_stream(self):
        while True:
            try:
                async for heading in self.drone.telemetry.heading():
                    self.telemetry_info_["heading"] = heading.heading_deg
            except Exception as e:
                self.log.error(f"heading stream koptu! 0.2 saniye bekleniyor... Hata: {e}")
                await asyncio.sleep(0.2)

    async def collect_velocity_stream(self):
        while True:
            try:
                async for velocity in self.drone.telemetry.velocity_ned():
                    north_m_s = float(getattr(velocity, "north_m_s", 0.0) or 0.0)
                    east_m_s = float(getattr(velocity, "east_m_s", 0.0) or 0.0)
                    down_m_s = float(getattr(velocity, "down_m_s", 0.0) or 0.0)
                    self.telemetry_info_["speed_m_s"] = math.sqrt(
                        north_m_s ** 2 + east_m_s ** 2 + down_m_s ** 2
                    )
            except Exception as e:
                self.log.error(f"velocity stream koptu! 0.2 saniye bekleniyor... Hata: {e}")
                await asyncio.sleep(0.2)

    async def collect_battery_stream(self):
        while True:
            try:
                async for battery in self.drone.telemetry.battery():
                    remaining_percent = getattr(battery, "remaining_percent", None)
                    if remaining_percent is None:
                        continue
                    battery_percent = float(remaining_percent)
                    if battery_percent <= 1.0:
                        battery_percent *= 100.0
                    self.telemetry_info_["battery_percent"] = max(
                        0.0,
                        min(100.0, battery_percent),
                    )
            except Exception as e:
                self.log.error(f"battery stream koptu! 0.2 saniye bekleniyor... Hata: {e}")
                await asyncio.sleep(0.2)

    async def run_collision_control(self):
        while True:
            if not self.is_follower_role():
                await asyncio.sleep(0.1)
                continue

            own_lat = self.telemetry_info_["latitude"]
            own_lon = self.telemetry_info_["longitude"]
            own_alt = self.telemetry_info_["absolute_altitude"]
            if own_lat is None or own_lon is None or own_alt is None:
                await asyncio.sleep(0.1)
                continue

            nearest_distance_m = None
            for partner_role, partner_info in self.swarm_infos_.items():
                if partner_role == self.role_str_:
                    continue

                partner_lat = partner_info["latitude"]
                partner_lon = partner_info["longitude"]
                partner_alt = partner_info["absolute_altitude"]
                if partner_lat is None or partner_lon is None or partner_alt is None:
                    continue

                horizontal_distance_m = nav.distance_to_target_m(
                    own_lat,
                    own_lon,
                    partner_lat,
                    partner_lon,
                )
                vertical_distance_m = abs(float(own_alt) - float(partner_alt))
                partner_distance_m = math.sqrt(
                    (horizontal_distance_m ** 2) + (vertical_distance_m ** 2)
                )
                if nearest_distance_m is None:
                    nearest_distance_m = partner_distance_m
                else:
                    nearest_distance_m = min(nearest_distance_m, partner_distance_m)

            if nearest_distance_m is None:
                await asyncio.sleep(0.1)
                continue

            if not self.is_collision_control_safe_:
                if nearest_distance_m > self.collision_settings_.release_distance_m:
                    self.is_collision_control_safe_ = True
            elif nearest_distance_m < self.collision_settings_.trigger_distance_m:
                self.is_collision_altitude_command_sent_ = False
                self.is_collision_control_safe_ = False
            await asyncio.sleep(0.1)

    def lider_kontrol_dongusu_wrapper(self):
        if not self.is_connected_ or self.lider_kontrol_busy_:
            return

        self.lider_kontrol_busy_ = True
        future = asyncio.run_coroutine_threadsafe(
            self.lider_kontrol_dongusu(),
            self.loop,
        )
        future.add_done_callback(self.lider_kontrol_done_callback)

    def lider_kontrol_done_callback(self, future):
        self.lider_kontrol_busy_ = False
        try:
            future.result()
        except Exception as e:
            self.log.error(f"lider_kontrol_done_callback fonksiyonunda hata: {e}")
    
    def takipci_kontrol_dongusu_wrapper(self):
        if not self.is_connected_ or self.takipci_kontrol_busy_:
            return

        self.takipci_kontrol_busy_ = True
        future = asyncio.run_coroutine_threadsafe(
            self.takipci_kontrol_dongusu(),
            self.loop,
        )
        future.add_done_callback(self.takipci_kontrol_done_callback)

    def takipci_kontrol_done_callback(self, future):
        self.takipci_kontrol_busy_ = False
        try:
            future.result()
        except Exception as e:
            self.log.error(f"takipci_kontrol_done_callback fonksiyonunda hata: {e}")

    def publish_swarm_info_wrapper(self):
        if not self.is_connected_ or self.leader_info_pub_busy_:
            return

        self.leader_info_pub_busy_ = True
        future = asyncio.run_coroutine_threadsafe(
            self.publish_swarm_info(),
            self.loop,
        )
        future.add_done_callback(self.publish_swarm_info_done_callback)

    def publish_swarm_info_done_callback(self, future):
        self.leader_info_pub_busy_ = False
        try:
            future.result()
        except Exception as e:
            self.log.error(f"publish_swarm_info_done_callback fonksiyonunda hata: {e}")

    def publish_swarm_info_once_done_callback(self, future):
        try:
            future.result()
        except Exception as e:
            self.log.error(f"publish_swarm_info_once_done_callback fonksiyonunda hata: {e}")

    def is_role_configured(self):
        return self.role_str_ is not None and self.own_slot_index_ is not None

    def is_swarm_info_telemetry_ready(self):
        required_fields = (
            "latitude",
            "longitude",
            "absolute_altitude",
            "relative_altitude",
            "heading",
        )
        return all(self.telemetry_info_.get(field) is not None for field in required_fields)

    async def publish_swarm_info(self):
        if (
            self.swarm_info_publisher_ is None
            or not self.is_role_configured()
            or not self.is_swarm_info_telemetry_ready()
        ):
            return

        is_follower = self.is_follower_role()
        is_leader = self.is_leader_role()

        packet = swarm_info.build_swarm_info_packet(
            sender_role=self.role_str_,
            telemetry_info=self.telemetry_info_,
            shared_target_reached=self.shared_target_reached_,
            all_mission_done=self.is_all_mission_done_,
            qr_event_seq=self.local_qr_event_seq_,
            qr_result=self.local_qr_result_,
            sender_udp_port=self.own_udp_port_,
            sender_drone_id=self.own_drone_id_,
            slot_index=self.own_slot_index_,
            state=self.state_.name,
            is_follower=is_follower,
            is_leader=is_leader,
            takeoff_done=self.is_takeoff_mission_done_,
            initial_positioning_done=self.is_initial_positioning_mission_done_,
            rotation_alignment_done=self.rotation_alignment_done_,
            swarm_departure_active=self.is_swarm_departure_active,
            formation_heading_ref=self.formation_heading_ref_,
            rotation_active=self.rotation_active_,
            rotating_done=self.is_rotating_mission_done_,
            leader_home_latitude=self.home_latitude_,
            leader_home_longitude=self.home_longitude_,
            leader_home_absolute_altitude=self.home_absolute_altitude_,
            field_coordinate_event_seq=self.field_coordinate_event_seq_,
            field_coordinates=self.field_coordinates_,
        )

        if is_leader:
            swarm_info.update_own_leader_swarm_info(
                leader_info=self.swarm_infos_["lider"],
                telemetry_info=self.telemetry_info_,
                formation_heading_ref=self.formation_heading_ref_,
                rotation_active=self.rotation_active_,
                shared_target_reached=self.shared_target_reached_,
                sender_udp_port=self.own_udp_port_,
                sender_drone_id=self.own_drone_id_,
                slot_index=self.own_slot_index_,
                state=self.state_.name,
            )

        msg = String()
        msg.data = json.dumps(packet)

        self.swarm_info_publisher_.publish(msg)

    def swarm_info_subscriber_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.log.error("swarm_info json parse hatası!")
            return

        if not self.is_role_configured() or not self.swarm_infos_:
            return

        sender_role = data.get("sender_role")
        if not sender_role:
            return

        if sender_role == self.role_str_:
            return

        if sender_role not in self.swarm_infos_:
            self.log.error(
                f"swarm_info_subscriber_callback'e plan disi rolde mesaj geldi: {sender_role}"
            )
            return

        if sender_role != "lider":
            swarm_info.update_follower_swarm_info(
                swarm_infos=self.swarm_infos_,
                sender_role=sender_role,
                data=data,
            )

        else:
            self.update_leader_swarm_info(data)

    def update_leader_swarm_info(self, data):
        # Liderden gelen mesaj, takipcilerin lider konumu ve ortak gorev
        # durumunu izlemesi icindir.
        swarm_info.update_leader_swarm_basic_info(self.swarm_infos_, data)

        (
            self.leader_home_latitude_,
            self.leader_home_longitude_,
            self.leader_home_absolute_altitude_,
        ) = swarm_info.read_leader_home_from_swarm_info(
            data=data,
            current_latitude=self.leader_home_latitude_,
            current_longitude=self.leader_home_longitude_,
            current_absolute_altitude=self.leader_home_absolute_altitude_,
        )
        (
            self.last_leader_field_coordinate_event_seq_,
            updated_color_names,
        ) = swarm_info.update_field_coordinates_from_leader(
            field_coordinates_state=self.field_coordinates_,
            data=data,
            last_event_seq=self.last_leader_field_coordinate_event_seq_,
        )

        for color_name in updated_color_names:
            self.log.info(
                f"Liderden {color_name} alan koordinati alindi: "
                f"{self.field_coordinates_[color_name]['latitude']}, "
                f"{self.field_coordinates_[color_name]['longitude']}"
            )

    def colored_field_subscriber_callback(self, msg):
        if self.role_ != DroneRole.LEADER:
            return
        if not self.is_leader_colored_field_detection_active():
            return

        try:
            colored_field_message = vision_qr.parse_colored_field_message(msg.data)
        except Exception as e:
            self.log.error(f"colored_field mesajı okunamadı: {e}")
            return

        if colored_field_message is None:
            return

        latitude = self.telemetry_info_["latitude"]
        longitude = self.telemetry_info_["longitude"]
        target_type = colored_field_message["target_type"]

        if (
            target_type in self.field_coordinates_
            and latitude is not None
            and longitude is not None
        ):
            self.field_coordinates_[target_type]["latitude"] = float(latitude)
            self.field_coordinates_[target_type]["longitude"] = float(longitude)
            self.field_coordinate_event_seq_ += 1
            if self.role_ == DroneRole.LEADER and self.is_connected_:
                future = asyncio.run_coroutine_threadsafe(
                    self.publish_swarm_info(),
                    self.loop,
                )
                future.add_done_callback(self.publish_swarm_info_once_done_callback)

        if target_type == "red":
            self.log.info(
                f"en yakin kirmizi alan koordinati: {latitude}, {longitude}"
            )
        elif target_type == "blue":
            self.log.info(
                f"en yakin mavi alan koordinati: {latitude}, {longitude}"
            )

        else:
            self.log.error("renkli alan tipi tanımlı değil!")

    def apply_formation_settings(self, formation_data):
        if not formation_data["aktif"]:
            self.formation_type_ = ""
            if self.is_follower_role():
                self.relative_angle_ = self.get_role_relative_angle(self.role_str_)
            return

        formation_type = formation_data["tip"]
        if formation_type not in FORMATION_ANGLES:
            self.log.error("bilinmeyen formasyon tipi")
            self.formation_type_ = ""
            return

        self.formation_type_ = formation_type
        relative_angle = self.get_role_relative_angle(self.role_str_)
        if relative_angle is not None:
            self.relative_angle_ = relative_angle

    def apply_maneuver_altitude(self, maneuver_data):
        self.mission_altitude_ = self.swarm_center_altitude_
        if not maneuver_data["aktif"]:
            self.maneuver_pitch_deg_ = 0.0
            self.maneuver_roll_deg_ = 0.0
            return

        role_name = self.role_str_

        self.maneuver_pitch_deg_ = float(maneuver_data.get("pitch_deg", 0.0))
        self.maneuver_roll_deg_ = float(maneuver_data.get("roll_deg", 0.0))
        maneuver_offset = self.get_maneuver_altitude_offset_m(role_name)

        self.mission_altitude_ = self.swarm_center_altitude_ + maneuver_offset

    def get_maneuver_altitude_offset_m(self, role_name):
        pitch_deg = self.maneuver_pitch_deg_
        roll_deg = self.maneuver_roll_deg_
        pitch_offset = 0.0
        roll_offset = 0.0

        if pitch_deg != 0.0:
            pitch_factor = PITCH_ALTITUDE_FACTORS.get(
                self.formation_type_,
                {},
            ).get(self.get_role_side_key(role_name), 0.0)
            if self.is_follower_role(role_name):
                pitch_factor *= self.get_role_slot_distance_scale(role_name)
            pitch_offset = pitch_factor * math.copysign(
                abs(
                    self.formation_settings_.follow_distance_m
                    * math.tan(math.radians(pitch_deg))
                )
                / 2.0,
                pitch_deg,
            )

        if roll_deg != 0.0:
            roll_factor = ROLL_ALTITUDE_FACTORS.get(
                self.get_role_side_key(role_name),
                0.0,
            )
            if self.is_follower_role(role_name):
                roll_factor *= self.get_role_slot_distance_scale(role_name)
            roll_offset = roll_factor * math.copysign(
                abs(
                    self.formation_settings_.follow_distance_m
                    * math.tan(math.radians(roll_deg))
                )
                / 2.0,
                roll_deg,
            )

        return pitch_offset + roll_offset

    def get_formation_horizontal_distance_m(self, role_name=None):
        role_name = role_name or self.role_str_
        follow_distance_m = (
            float(self.formation_settings_.follow_distance_m)
            * self.get_role_slot_distance_scale(role_name)
        )
        altitude_diff_m = abs(
            self.get_maneuver_altitude_offset_m(role_name)
            - self.get_maneuver_altitude_offset_m("lider")
        )
        return math.sqrt(
            max((follow_distance_m ** 2) - (altitude_diff_m ** 2), 0.0)
        )

    def new_mission_subscriber_callback_wrapper(self, msg):
        asyncio.run_coroutine_threadsafe(
            self.new_mission_subscriber_callback(msg),
            self.loop,
        )

    async def new_mission_subscriber_callback(self, msg):
        # Yeni görev QR'dan lider tarafından yayınlanır ve tüm drone'lar buradan okur.
        try:
            mission_data = json.loads(msg.data)
            self.log.info(f"yeni plan:\n{mission_data}")
        except Exception as e:
            self.log.error(f"yeni plan alırken bir sorun meydana geldi: {e}")
            return

        # Mevcut görev bitmeden yeni görev ayarlarına geçilmez.
        while not self.is_all_mission_done_:
            await asyncio.sleep(0.2)

        if self.is_follower_role():
            self.log.info("takipçi görevi aldı. bilinçli bekleme yapıyor.")
            await asyncio.sleep(1)

        gorev_data = mission_data["gorev"]
        formasyon_data = gorev_data["formasyon"]
        manevra_data = gorev_data["manevra_pitch_roll"]
        irtifa_degisimi_data = gorev_data["irtifa_degisimi"]
        suruden_ayrilma_data = mission_data["suruden_ayrilma"]
        sonraki_qr_data = mission_data["sonraki_qr"]

        # sonraki_qr=0 final görev işaretidir; bu durumda hedef home tarafına döner.
        if isinstance(sonraki_qr_data, set):
            self.return_home_after_current_mission_ = sonraki_qr_data == {0}
        else:
            if isinstance(sonraki_qr_data, str):
                sonraki_qr_data = sonraki_qr_data.strip()
            self.return_home_after_current_mission_ = sonraki_qr_data in (0, 0.0, "0")
        self.shared_target_reached_ = False
        self.reset_return_home_sequence()

        # Sürüden ayrılacak takipçi drone'u belirle.
        departure_role = None
        if bool(suruden_ayrilma_data.get("aktif", False)):
            raw_departure_id = suruden_ayrilma_data.get("ayrilacak_drone_id")
            try:
                departure_id = int(raw_departure_id)
            except (TypeError, ValueError):
                self.log.error(
                    f"Sürüden ayrılacak drone id geçersiz: {raw_departure_id}"
                )
            else:
                departure_role = self.role_key_by_drone_id_.get(departure_id)
                if departure_role is None:
                    self.log.error(
                        f"Sürüden ayrılacak drone id bilinmiyor: {departure_id}"
                    )
                elif departure_role == "lider":
                    self.log.error(
                        "Lider için sürüden ayrılma/hassas iniş bu akışta desteklenmiyor."
                    )
                    departure_role = None

        self.is_swarm_departure_active = (
            departure_role is not None
            and self.role_str_ == departure_role
        )
        if self.is_follower_role():
            self.swarm_infos_[self.role_str_]["swarm_departure_active"] = bool(
                self.is_swarm_departure_active
            )

        if not self.is_swarm_departure_active:
            self.target_color_ = None
        else:
            target_color = suruden_ayrilma_data.get("hedef_renk")
            self.target_color_ = (
                str(target_color).strip().lower()
                if target_color is not None
                else None
            )
            if not self.target_color_:
                self.target_color_ = None
                self.is_swarm_departure_active = False
                if self.is_follower_role():
                    self.swarm_infos_[self.role_str_]["swarm_departure_active"] = bool(
                        self.is_swarm_departure_active
                    )
                self.log.error("Sürüden ayrılma aktif ama hedef renk verilmedi.")

        # Görev ayarlarını sırayla uygula.
        self.apply_formation_settings(formasyon_data)
        if irtifa_degisimi_data["aktif"]:
            self.swarm_center_altitude_ = (
                float(irtifa_degisimi_data["deger"]) + self.home_absolute_altitude_
            )
        self.apply_maneuver_altitude(manevra_data)
        self.original_mission_altitude_ = self.mission_altitude_

        if not self.set_final_return_home_target_if_needed():
            self.target_positions_["lider"]["latitude"] = float(
                mission_data["sonraki_qr"]["latitude"]
            )
            self.target_positions_["lider"]["longitude"] = float(
                mission_data["sonraki_qr"]["longitude"]
            )

        # Yeni göreve temiz görev bayraklarıyla başla.
        self.is_rotating_mission_done_ = False
        self.is_initial_positioning_mission_done_ = False
        self.is_all_mission_done_ = False
        self.reset_navigation_session_state()
        self.reset_land_to_area_session_state(clear_xy_error=True)
        self.local_qr_result_ = None
        self.reset_qr_wait_state(disable_scanner=True, reason="new_mission")
        self.publish_vision_control()
        self.logged_events_.clear()

    def x_y_error_callback(self, msg):
        try:
            x_y_error_message = vision_qr.parse_x_y_error_message(msg.data)
        except Exception as e:
            self.log.error(f"x,y error datası alınamadı! Hata: {e}")
            return

        if x_y_error_message is None:
            return

        target_type = x_y_error_message["target_type"]
        if self.target_color_ is not None and target_type != self.target_color_:
            return

        self.x_y_error_["x"] = float(x_y_error_message["x"])
        self.x_y_error_["y"] = float(x_y_error_message["y"])
        self.x_y_error_["detected"] = bool(x_y_error_message["detected"])
        self.x_y_error_["type"] = target_type

    def is_control_loop_ready(self):
        return self.is_connected_ and self.gcs_mission_plan_ is not None

    async def dispatch_state_handler(self, state_handlers):
        handler = state_handlers.get(self.state_)
        if handler is not None:
            await handler()

    async def recover_control_loop_error(self, message, stop_reason):
        self.log.error(message)
        if self.nav_offboard_active_:
            if not await self.confirm_ned_navigation_stopped(stop_reason):
                return
        self.state_ = DroneState.IDLE

    async def lider_kontrol_dongusu(self):
        if not self.is_control_loop_ready():
            return

        if self.is_emergency_land_active:
            self.log_once("Acil iniş yapılıyor...")
            return

        # Liderin bir sonraki state'ini belirle ve o state'in handler'ini calistir.
        self.lider_state_guncelle()
        self.update_leader_qr_wait_state()

        try:
            state_handlers = {
                DroneState.IDLE: self.handle_lider_idle,
                DroneState.TAKE_OFF: self.handle_lider_takeoff,
                DroneState.RETURN_TO_HOME: self.run_return_to_home,
                DroneState.ROTATION: self.handle_lider_rotation,
                DroneState.MOVE: self.handle_lider_move,
            }
            handler = state_handlers.get(self.state_)
            if handler is not None:
                await handler()
        except Exception as e:
            await self.recover_control_loop_error(
                f"lider kontrol döngüsünde hata: {e}",
                "Lider kontrol dongusu hata sonrasi",
            )

    async def handle_lider_takeoff(self):
        if not await self.run_arm_step():
            return

        if self.is_takeoff_mission_done_:
            return

        await self.run_takeoff_step()

    def are_followers_takeoff_done(self):
        return self.are_all_followers_flag_true("takeoff_done")

    def are_followers_initial_positioning_done(self):
        return self.are_all_followers_flag_true("initial_positioning_done")

    def should_prepare_leader_altitude_before_rotation(self):
        if self.role_ != DroneRole.LEADER:
            return False
        if not self.is_takeoff_mission_done_ or self.is_all_mission_done_:
            return False
        if self.is_return_to_home_after_mission_ or self.shared_target_reached_:
            return False
        if self.mission_altitude_ is None or self.formation_heading_ref_ is None:
            return False
        if not self.are_followers_takeoff_done():
            return False
        if (
            self.are_followers_initial_positioning_done()
            and self.leader_altitude_prepared_
        ):
            return False
        return not self.is_rotating_mission_done_

    async def handle_lider_idle(self):
        if self.should_prepare_leader_altitude_before_rotation():
            await self.prepare_leader_altitude_before_rotation()
            return

        await self.hold_ned_navigation()

    async def prepare_leader_altitude_before_rotation(self):
        # Takipciler formasyona gecmeden once lider gorev irtifasina cikar/iner.
        current_lat = self.telemetry_info_["latitude"]
        current_lon = self.telemetry_info_["longitude"]
        current_heading = self.telemetry_info_["heading"]
        if (
            current_lat is None
            or current_lon is None
            or current_heading is None
            or self.mission_altitude_ is None
        ):
            await self.hold_ned_navigation()
            return

        try:
            north_m_s, east_m_s, down_m_s, _, down_error_m = (
                self.make_point_velocity_ned_command(
                    current_lat,
                    current_lon,
                    self.mission_altitude_,
                    0.0,
                )
            )
            await self.send_velocity_ned_with_offboard(
                north_m_s,
                east_m_s,
                down_m_s,
                current_heading,
            )
        except Exception as e:
            self.log.error(f"Lider irtifa hazirligi NED komutu hatasi: {e}")
            await self.stop_ned_navigation()
            return

        if not self.leader_altitude_prepare_command_sent_:
            self.log.info(
                "Lider yönelimini değiştirmeden görev irtifasına hazırlanıyor -> "
                f"alt:{self.mission_altitude_:.2f}"
            )
            self.leader_altitude_prepare_command_sent_ = True

        self.leader_altitude_prepared_ = (
            abs(down_error_m) <= self.nav_settings_.alt_tolerance_m
        )
        if self.leader_altitude_prepared_:
            self.log_once(
                "Lider görev irtifasına ulaştı, takipçilerin formasyonunu bekliyor.",
                "lider_irtifa_hazir",
            )

    async def handle_lider_rotation(self):
        # Lider rotasyon sirasinda konumunu ve gorev irtifasini korur.
        current_lat = self.telemetry_info_["latitude"]
        current_lon = self.telemetry_info_["longitude"]
        current_abs_alt = self.telemetry_info_["absolute_altitude"]
        current_heading = self.telemetry_info_["heading"]

        if (
            current_lat is None
            or current_lon is None
            or current_abs_alt is None
            or current_heading is None
            or self.mission_altitude_ is None
        ):
            await self.set_idle_after_ned_stop("Lider rotasyon telemetri/hedef eksigi")
            return

        # Rotasyon hedef acisi ilk giriste hesaplanir, sonraki dongulerde korunur.
        if self.rotation_target_heading_ is None:
            target_lat = self.target_positions_["lider"]["latitude"]
            target_lon = self.target_positions_["lider"]["longitude"]
            if target_lat is not None and target_lon is not None:
                self.rotation_target_heading_ = nav.calculate_bearing(
                    float(current_lat),
                    float(current_lon),
                    float(target_lat),
                    float(target_lon),
                    bearing_offset_deg=self.nav_settings_.bearing_offset_deg,
                )
            else:
                self.rotation_target_heading_ = float(current_heading)

        self.log_once(
            "Lider toplu rotasyonu başlattı. "
            f"Hedef Açı: {self.rotation_target_heading_:.2f}°",
            "rotating",
        )

        self.rotation_active_ = True
        now_s = time.monotonic()

        # Heading bir anda degil, belirlenen hizla kademeli olarak hedefe yaklastirilir.
        rotation_update_due = (
            not self.is_rotation_command_sent_
            or (now_s - self.rotation_last_update_)
            >= self.rotation_settings_.command_period_s
        )
        if rotation_update_due:
            dt_s = self.rotation_settings_.command_period_s
            if self.rotation_last_update_ != 0.0:
                dt_s = max(
                    now_s - self.rotation_last_update_,
                    self.rotation_settings_.command_period_s,
                )
            stepped_heading = nav.step_towards_heading(
                current_heading,
                self.rotation_target_heading_,
                self.rotation_settings_.leader_rate_deg_s * dt_s,
            )
            self.formation_heading_ref_ = stepped_heading

            try:
                north_m_s, east_m_s, down_m_s, _, _ = (
                    self.make_point_velocity_ned_command(
                        current_lat,
                        current_lon,
                        self.mission_altitude_,
                        self.nav_settings_.rotation_xy_max_m_s,
                    )
                )
                await self.send_velocity_ned_with_offboard(
                    north_m_s,
                    east_m_s,
                    down_m_s,
                    stepped_heading,
                )
                self.is_rotation_command_sent_ = True
                self.rotation_last_update_ = now_s
            except Exception as e:
                self.log.error(f"NED yönelim aşamasında hata meydana geldi! Hata:{e}")
                await self.stop_ned_navigation()
                return

        heading_difference = nav.get_heading_difference(
            current_heading,
            self.rotation_target_heading_,
        )
        if heading_difference >= self.rotation_settings_.heading_tolerance_deg:
            return

        self.formation_heading_ref_ = nav.normalize_heading(
            self.rotation_target_heading_
        )
        followers_aligned = self.are_all_followers_flag_true("rotation_alignment_done")
        if not followers_aligned:
            return

        self.rotation_active_ = False
        self.is_rotation_command_sent_ = False
        self.rotation_last_update_ = 0.0
        self.log.info("Lider ve takipçiler toplu yönelimini tamamladı.")
        self.is_rotating_mission_done_ = True

    async def handle_lider_move(self):
        # Lider hedef koordinata giderken kendi heading'ini hedef yonune gore gunceller.
        current_lat = self.telemetry_info_["latitude"]
        current_lon = self.telemetry_info_["longitude"]
        current_heading = self.telemetry_info_["heading"]
        target_lat = self.target_positions_["lider"]["latitude"]
        target_lon = self.target_positions_["lider"]["longitude"]
        if (
            current_lat is None
            or current_lon is None
            or current_heading is None
            or target_lat is None
            or target_lon is None
            or self.mission_altitude_ is None
        ):
            await self.set_idle_after_ned_stop("Lider move telemetri/hedef eksigi")
            return

        try:
            target_reached = self.is_global_target_reached(
                target_lat,
                target_lon,
                self.mission_altitude_,
                arrival_radius_m=self.nav_settings_.final_arrival_radius_m,
            )
        except ValueError:
            await self.stop_ned_navigation()
            return

        if target_reached:
            if not await self.confirm_ned_navigation_held("Lider hedefe varis"):
                return
            self.log.info("Hedef noktaya varıldı.")
            self.move_command_sent_ = False
            self.shared_target_reached_ = True
            self.clear_logged_events(
                "Final gorevde tum drone'larin ortak hedef formasyonuna ulasmasi bekleniyor..."
            )
            if self.return_home_after_current_mission_:
                self.log.info(
                    "Final görev ortak formasyon hedefi tamamlandı. Diğer drone'lar bekleniyor."
                )
                return

            self.is_all_mission_done_ = True
            return

        try:
            north_m_s, east_m_s, down_m_s, horizontal_error_m, _ = (
                self.make_point_velocity_ned_command(
                    target_lat,
                    target_lon,
                    self.mission_altitude_,
                    self.nav_settings_.leader_xy_max_m_s,
                )
            )
        except ValueError as e:
            self.log.error(f"Lider NED hareketi icin hedef hatasi hesaplanamadi: {e}")
            await self.set_idle_after_ned_stop("Lider move NED hedef hatasi")
            return

        target_heading = current_heading
        if horizontal_error_m > self.nav_settings_.arrival_radius_m:
            target_heading = nav.calculate_bearing(
                current_lat,
                current_lon,
                target_lat,
                target_lon,
                bearing_offset_deg=self.nav_settings_.bearing_offset_deg,
            )

        if not self.move_command_sent_:
            self.log.info(
                f"Hedefe NED hiz kontroluyle gidiliyor -> lat:{target_lat}, lon:{target_lon}"
            )

        try:
            await self.send_velocity_ned_with_offboard(
                north_m_s,
                east_m_s,
                down_m_s,
                target_heading,
            )
            self.move_command_sent_ = True
        except Exception as e:
            self.log.error(f"Lider NED move komutu hatası: {e}")
            await self.stop_ned_navigation()
            return

    def lider_state_guncelle(self):
        followers_takeoff_done = self.are_followers_takeoff_done()
        followers_initial_positioning_done = (
            self.are_followers_initial_positioning_done()
        )
        all_shared_targets_reached = self.are_all_roles_shared_target_reached()

        # 1. Lider once kendi kalkisini tamamlar.
        if not self.is_takeoff_mission_done_:
            self.state_ = DroneState.TAKE_OFF
            return

        # 2. Final donus basladiysa lider sadece RETURN_TO_HOME state'ine gecer.
        if self.is_return_to_home_after_mission_ and not self.is_all_mission_done_:
            self.state_ = DroneState.RETURN_TO_HOME
            return

        # 3. Gorev bitmisse lider bekleme modunda kalir.
        if self.is_all_mission_done_:
            self.rotation_active_ = False
            self.log_once("tüm görevler tamamlandı. yeni görev bekleniyor...")
            self.state_ = DroneState.IDLE
            return

        if not followers_takeoff_done:
            self.rotation_active_ = False
            self.log_once("lider takipçilerin takeoff görevini bitirmesini bekliyor...")
            self.state_ = DroneState.IDLE
            return

        self.clear_logged_events(
            "lider takipçilerin takeoff görevini bitirmesini bekliyor..."
        )

        # 4. Formasyon icin ilk heading referansi liderin mevcut yonunden alinir.
        if self.formation_heading_ref_ is None:
            if self.telemetry_info_["heading"] is not None:
                self.formation_heading_ref_ = nav.normalize_heading(
                    self.telemetry_info_["heading"]
                )
                self.rotation_active_ = False
                self.log.info(
                    "Lider ilk formasyon heading referansini kilitledi:"
                    f"{self.formation_heading_ref_:.2f}°"
                )
            self.state_ = DroneState.IDLE
            return

        if not followers_initial_positioning_done:
            self.rotation_active_ = False
            self.log_once(
                "lider takipçilerin ilk konumlanma görevini bitirmesini bekliyor..."
            )
            self.state_ = DroneState.IDLE
            return

        self.clear_logged_events(
            "lider takipçilerin ilk konumlanma görevini bitirmesini bekliyor..."
        )

        # 5. Takipciler hazir olduktan sonra lider gorev irtifasini tamamlar.
        if not self.leader_altitude_prepared_:
            self.rotation_active_ = False
            self.log_once(
                "takipçiler hazır, lider görev irtifasına ulaşmayı bekliyor...",
                "lider_irtifa_bekleniyor",
            )
            self.state_ = DroneState.IDLE
            return

        self.clear_logged_events("lider_irtifa_bekleniyor")

        # 6. Irtifa hazirsa toplu rotasyon baslar.
        if not self.is_rotating_mission_done_:
            self.rotation_active_ = True
            self.state_ = DroneState.ROTATION
            return

        self.rotation_active_ = False

        # 7. Final gorevde herkes ortak hedefe ulasinca return-home state'i baslar.
        if self.shared_target_reached_:
            if self.return_home_after_current_mission_:
                if not all_shared_targets_reached:
                    self.log_once(
                        "Final gorevde tum drone'larin ortak hedef "
                        "formasyonuna ulasmasi bekleniyor..."
                    )
                    self.state_ = DroneState.IDLE
                    return

                self.is_return_to_home_after_mission_ = True
                self.state_ = DroneState.RETURN_TO_HOME
                return

            self.state_ = DroneState.IDLE
            return

        self.log_once(
            f"Hedefe gidiliyor -> lat:{self.target_positions_['lider']['latitude']}, "
            f"lon:{self.target_positions_['lider']['longitude']}",
            "hedefe_gidiliyor",
        )
        self.state_ = DroneState.MOVE

    async def run_arm_step(self, use_action_result_string=False):
        if self.is_arm_mission_done_:
            return True

        if not self.arm_command_sent_:
            self.log.info("Drone'a arm komutu gönderildi.")
            try:
                await self.drone.action.arm()
            except ActionError as e:
                error_detail = e._result_str if use_action_result_string else e
                self.log.error(f"arm ederken hata: {error_detail}")
                self.log.info("Ajan IDLE moduna alındı.")
                self.state_ = DroneState.IDLE

            self.arm_command_sent_ = True

        async for is_armed in self.drone.telemetry.armed():
            if is_armed:
                self.log.info("Drone arm edildi.")
                self.arm_command_sent_ = False
                self.is_arm_mission_done_ = True
                break
            break
        return False

    async def run_takeoff_step(self, reset_arm_command_on_completion=False):
        if not self.takeoff_command_sent_:
            self.takeoff_command_sent_ = True
            try:
                await self.drone.action.set_takeoff_altitude(self.takeoff_altitude_)
                await self.drone.action.takeoff()
                self.log.info(f"Kalkış yapılıyor... Hedef: {self.takeoff_altitude_}m")
            except ActionError as e:
                self.log.error(f"Takeoff hatası: {e._result_str}")
                self.state_ = DroneState.IDLE
                return

        if self.telemetry_info_["relative_altitude"] >= self.takeoff_altitude_ - 0.5:
            self.log.info("Takeoff tamamlandı.")
            self.log.info("3 saniye bekleniyor...")
            await asyncio.sleep(3)
            if reset_arm_command_on_completion:
                self.arm_command_sent_ = False
            self.takeoff_command_sent_ = False
            self.is_takeoff_mission_done_ = True

    def are_all_roles_shared_target_reached(self):
        all_reached = bool(self.shared_target_reached_)
        for role_name in self.swarm_infos_:
            if role_name != self.role_str_:
                all_reached = all_reached and bool(
                    self.swarm_infos_[role_name]["shared_target_reached"]
                )
        return all_reached

    def get_leader_formation_heading_ref(self):
        if self.role_ == DroneRole.LEADER and self.formation_heading_ref_ is not None:
            return self.formation_heading_ref_
        return self.swarm_infos_["lider"]["formation_heading_ref"]

    async def takipci_kontrol_dongusu(self):
        if not self.is_control_loop_ready():
            return

        if self.is_emergency_land_active:
            self.log_once("Acil iniş yapılıyor...")
            return

        # Takipcinin bir sonraki state'ini belirle; bekleme gerekiyorsa hover'da kalir.
        if not self.takipci_state_guncelle():
            await self.hold_ned_navigation()
            return

        try:
            state_handlers = {
                DroneState.IDLE: self.hold_ned_navigation,
                DroneState.TAKE_OFF: self.handle_takipci_takeoff,
                DroneState.RETURN_TO_HOME: self.run_return_to_home,
                DroneState.INITIAL_POSITIONING: self.handle_takipci_initial_positioning,
                DroneState.ROTATION: self.handle_takipci_rotation,
                DroneState.MOVE: self.handle_takipci_move,
                DroneState.GOTO_AREA: self.handle_takipci_goto_area,
                DroneState.LAND_TO_AREA: self.handle_takipci_land_to_area,
            }
            handler = state_handlers.get(self.state_)
            if handler is not None:
                await handler()
        except Exception as e:
            await self.recover_control_loop_error(
                f"takipçi kontrol döngüsünde hata: {e}",
                "Takipci kontrol dongusu hata sonrasi",
            )

    async def handle_takipci_takeoff(self):
        if not await self.run_arm_step(use_action_result_string=True):
            return

        if not self.is_takeoff_mission_done_:
            await self.run_takeoff_step(reset_arm_command_on_completion=True)
            return

        self.log.error("Takipçi arm veya takeoff akışına girmedi.")

    async def handle_takipci_initial_positioning(self):
        # Takipci, lider konumu ve formasyon acisina gore ilk slotuna gider.
        self.rotation_alignment_done_ = False

        leader_latitude = self.swarm_infos_["lider"]["latitude"]
        leader_longitude = self.swarm_infos_["lider"]["longitude"]
        formation_heading = self.get_leader_formation_heading_ref()
        relative_angle = self.get_role_relative_angle()

        if (
            leader_latitude is None
            or leader_longitude is None
            or formation_heading is None
            or relative_angle is None
        ):
            self.log.error("formasyon heading veya relatif aci hesaplanamadi.")
            await self.set_idle_after_ned_stop(
                "Ilk konumlanma lider/formasyon hedef eksigi"
            )
            return

        try:
            self.update_target_positions(reference_heading_deg=formation_heading)
        except Exception as e:
            self.log.error(f"hedef pozisyonlar belirlenirken bir hata meydana geldi: {e}")
            await self.set_idle_after_ned_stop("Ilk konumlanma hedef pozisyon hatasi")
            return

        target_lat, target_lon = nav.offset_coordinate(
            float(leader_latitude),
            float(leader_longitude),
            formation_heading,
            relative_angle,
            self.get_formation_horizontal_distance_m(),
        )

        # Cok yakinlasma riski varsa takipci once gecici bir irtifa slotuna cikar/iner.
        if not self.is_collision_control_safe_:
            altitude_offset = self.collision_settings_.avoidance_altitude_offset_m
            altitude_action_text = "Yükselme"
            if self.role_ == DroneRole.SAG:
                altitude_offset = -self.collision_settings_.avoidance_altitude_offset_m
                altitude_action_text = "Alçalma"

            self.collision_control_mission_altitude_ = (
                self.original_mission_altitude_ + altitude_offset
            )
            self.mission_altitude_ = self.collision_control_mission_altitude_

            instant_lat = self.telemetry_info_["latitude"]
            instant_lon = self.telemetry_info_["longitude"]
            current_heading = self.telemetry_info_["heading"]
            if instant_lat is None or instant_lon is None or current_heading is None:
                await self.set_idle_after_ned_stop("Carpisma onleme telemetri eksigi")
                return

            try:
                north_m_s, east_m_s, down_m_s, _, down_error_m = (
                    self.make_point_velocity_ned_command(
                        instant_lat,
                        instant_lon,
                        self.mission_altitude_,
                        0.0,
                    )
                )
                await self.send_velocity_ned_with_offboard(
                    north_m_s,
                    east_m_s,
                    down_m_s,
                    current_heading,
                )
            except Exception as e:
                self.log.error(f"{altitude_action_text} NED komutu gonderilemedi: {e}")
                await self.stop_ned_navigation()
                return

            if not self.is_collision_altitude_command_sent_:
                self.log.info(f"{altitude_action_text} emri verildi")
                self.is_collision_altitude_command_sent_ = True
                self.is_initial_positioning_command_sent_ = False

            if abs(down_error_m) <= self.nav_settings_.alt_tolerance_m:
                self.log_once(
                    f"{altitude_action_text} gerçekleşti.",
                    f"collision_altitude_reached_{self.role_str_}",
                )
            return

        if self.collision_control_mission_altitude_ is not None:
            current_lat = self.telemetry_info_["latitude"]
            current_lon = self.telemetry_info_["longitude"]
            if current_lat is None or current_lon is None:
                await self.set_idle_after_ned_stop(
                    "Carpisma onleme slot mesafesi telemetri eksigi"
                )
                return

            slot_distance_m = nav.distance_to_target_m(
                current_lat,
                current_lon,
                target_lat,
                target_lon,
            )

            if slot_distance_m <= (
                self.collision_settings_.altitude_release_slot_distance_m
            ):
                self.mission_altitude_ = self.original_mission_altitude_
                self.collision_control_mission_altitude_ = None
                self.is_collision_altitude_command_sent_ = False
                self.is_initial_positioning_command_sent_ = False
            else:
                self.mission_altitude_ = self.collision_control_mission_altitude_
        else:
            self.mission_altitude_ = self.original_mission_altitude_

        # Normal ilk konumlanma komutu.
        if not self.is_initial_positioning_command_sent_:
            self.log.info("initial positioning aşamasına geçildi.")

        try:
            north_m_s, east_m_s, down_m_s, _, _ = (
                self.make_point_velocity_ned_command(
                    target_lat,
                    target_lon,
                    self.mission_altitude_,
                    self.nav_settings_.initial_positioning_xy_max_m_s,
                )
            )
            await self.send_velocity_ned_with_offboard(
                north_m_s,
                east_m_s,
                down_m_s,
                formation_heading,
            )
            if not self.is_initial_positioning_command_sent_:
                self.log.info("initial positioning NED komutu gönderildi.")
            self.is_initial_positioning_command_sent_ = True
        except Exception as e:
            self.log.error(
                f"initial positioning NED komutu gönderilirken hata meydana geldi: {e}"
            )
            self.is_initial_positioning_command_sent_ = False
            await self.stop_ned_navigation()
            return

        # Konum, irtifa ve heading ayni anda tolerans icindeyse ilk konumlanma biter.
        heading_difference = float("inf")
        if self.telemetry_info_["heading"] is not None:
            heading_difference = nav.get_heading_difference(
                self.telemetry_info_["heading"],
                formation_heading,
            )

        try:
            target_reached = self.is_global_target_reached(
                target_lat,
                target_lon,
                self.mission_altitude_,
                arrival_radius_m=self.nav_settings_.arrival_radius_m,
            )
        except ValueError:
            await self.stop_ned_navigation()
            return
        if not target_reached:
            return
        if heading_difference >= self.rotation_settings_.heading_tolerance_deg:
            return
        if not self.is_collision_control_safe_:
            return

        self.log.info("İlk konumlanma başarılı.")
        self.log.info("3 saniye hover bekleniyor...")
        if not await self.hold_ned_navigation_for(3.0, formation_heading):
            return
        self.log.info("3 saniyelik hover beklemesi tamamlandi.")
        self.is_initial_positioning_mission_done_ = True
        self.takeoff_command_sent_ = False
        self.rotation_alignment_done_ = False
        self.is_rotation_command_sent_ = False
        self.rotation_last_update_ = 0.0

    async def handle_takipci_rotation(self):
        # Lider donerken takipci kendi formasyon slotunu ve heading'ini gunceller.
        leader_latitude = self.swarm_infos_["lider"]["latitude"]
        leader_longitude = self.swarm_infos_["lider"]["longitude"]
        formation_heading = self.get_leader_formation_heading_ref()
        relative_angle = self.get_role_relative_angle()

        if leader_latitude is None:
            self.rotation_alignment_done_ = False
            await self.set_idle_after_ned_stop("Takipci rotasyon lider latitude eksigi")
            return
        if leader_longitude is None:
            self.rotation_alignment_done_ = False
            await self.set_idle_after_ned_stop("Takipci rotasyon lider longitude eksigi")
            return
        if formation_heading is None:
            self.rotation_alignment_done_ = False
            await self.set_idle_after_ned_stop("Takipci rotasyon formasyon heading eksigi")
            return
        if relative_angle is None:
            self.rotation_alignment_done_ = False
            await self.set_idle_after_ned_stop("Takipci rotasyon relatif aci eksigi")
            return

        self.mission_altitude_ = self.original_mission_altitude_

        target_lat, target_lon = nav.offset_coordinate(
            float(leader_latitude),
            float(leader_longitude),
            float(formation_heading),
            relative_angle,
            self.get_formation_horizontal_distance_m(),
        )

        try:
            north_m_s, east_m_s, down_m_s, _, _ = (
                self.make_point_velocity_ned_command(
                    target_lat,
                    target_lon,
                    self.mission_altitude_,
                    self.nav_settings_.rotation_xy_max_m_s,
                )
            )
            await self.send_velocity_ned_with_offboard(
                north_m_s,
                east_m_s,
                down_m_s,
                formation_heading,
            )
            self.is_rotation_command_sent_ = True
            self.rotation_last_update_ = time.monotonic()
        except Exception as e:
            self.rotation_alignment_done_ = False
            self.log.error(
                f"{self.role_str_} drone rotasyon slotu NED ile guncellenirken hata meydana geldi! "
                f"Hata: {e}"
            )
            await self.stop_ned_navigation()
            return

        heading_difference = float("inf")
        if self.telemetry_info_["heading"] is not None:
            heading_difference = nav.get_heading_difference(
                self.telemetry_info_["heading"],
                formation_heading,
            )

        try:
            target_reached = self.is_global_target_reached(
                target_lat,
                target_lon,
                self.mission_altitude_,
                arrival_radius_m=self.nav_settings_.arrival_radius_m,
            )
        except ValueError:
            self.rotation_alignment_done_ = False
            await self.stop_ned_navigation()
            return

        self.rotation_alignment_done_ = bool(
            target_reached
            and heading_difference < self.rotation_settings_.heading_tolerance_deg
        )

    async def handle_takipci_move(self):
        # Takipci, liderin hedefe gidis yonunu kullanarak kendi nihai slotunu takip eder.
        leader_latitude = self.swarm_infos_["lider"]["latitude"]
        leader_longitude = self.swarm_infos_["lider"]["longitude"]
        track_heading = self.swarm_infos_["lider"]["heading"]
        leader_target_reached = self.swarm_infos_["lider"]["shared_target_reached"]
        target_lat = self.target_positions_["lider"]["latitude"]
        target_lon = self.target_positions_["lider"]["longitude"]
        if leader_target_reached:
            stable_heading = self.get_leader_formation_heading_ref()
            if stable_heading is not None:
                track_heading = stable_heading
        elif (
            leader_latitude is not None
            and leader_longitude is not None
            and target_lat is not None
            and target_lon is not None
        ):
            track_heading = nav.calculate_bearing(
                float(leader_latitude),
                float(leader_longitude),
                float(target_lat),
                float(target_lon),
                bearing_offset_deg=self.nav_settings_.bearing_offset_deg,
            )

        relative_angle = self.get_role_relative_angle()
        if track_heading is None or relative_angle is None:
            self.log.error("Sürü hareketi için heading veya relatif açı hesaplanamadı.")
            await self.set_idle_after_ned_stop("Takipci move heading/relatif aci eksigi")
            return
        if leader_latitude is None or leader_longitude is None:
            self.log.error("Sürü hareketi için lider konumu hesaplanamadı.")
            await self.set_idle_after_ned_stop("Takipci move lider konumu eksigi")
            return

        self.update_target_positions(reference_heading_deg=track_heading)

        final_target_lat = self.target_positions_[self.role_str_]["latitude"]
        final_target_lon = self.target_positions_[self.role_str_]["longitude"]
        if (
            final_target_lat is None
            or final_target_lon is None
            or self.mission_altitude_ is None
        ):
            await self.set_idle_after_ned_stop("Takipci move final hedef eksigi")
            return

        try:
            final_target_reached = self.is_global_target_reached(
                final_target_lat,
                final_target_lon,
                self.mission_altitude_,
                arrival_radius_m=self.nav_settings_.final_arrival_radius_m,
            )
        except ValueError:
            await self.stop_ned_navigation()
            return

        if final_target_reached:
            if not await self.confirm_ned_navigation_held("Takipci final hedefe varis"):
                return
            self.move_command_sent_ = False
            self.log.info("Hedef noktaya varıldı.")
            self.shared_target_reached_ = True
            self.clear_logged_events(
                "Final gorevde tum drone'larin ortak hedef formasyonuna ulasmasi bekleniyor..."
            )
            if not self.return_home_after_current_mission_:
                self.is_all_mission_done_ = True
            else:
                self.log.info(
                    "Final görev ortak formasyon hedefi tamamlandı. Diğer drone'lar bekleniyor."
                )
            return

        if leader_target_reached:
            leader_ff_north_m_s = 0.0
            leader_ff_east_m_s = 0.0
            instant_target_latitude = final_target_lat
            instant_target_longitude = final_target_lon
        else:
            # Lider hizina gore kisa vadeli lider konumu tahmin edilir.
            leader_ff_north_m_s, leader_ff_east_m_s, _ = (
                nav.make_xy_velocity_from_global_points(
                    leader_latitude,
                    leader_longitude,
                    target_lat,
                    target_lon,
                    self.nav_settings_.leader_xy_max_m_s,
                    self.nav_settings_.kp_xy,
                    self.nav_settings_.slow_radius_m,
                    self.nav_settings_.hold_speed_epsilon_m_s,
                )
            )
            predicted_leader_latitude, predicted_leader_longitude = (
                nav.add_ned_offset_to_coordinate(
                    leader_latitude,
                    leader_longitude,
                    leader_ff_north_m_s * self.nav_settings_.leader_state_latency_comp_s,
                    leader_ff_east_m_s * self.nav_settings_.leader_state_latency_comp_s,
                )
            )
            instant_target_latitude, instant_target_longitude = nav.offset_coordinate(
                predicted_leader_latitude,
                predicted_leader_longitude,
                track_heading,
                relative_angle,
                self.get_formation_horizontal_distance_m(),
            )

        if not self.move_command_sent_:
            self.log.info(
                f"Hedefe NED hiz kontroluyle gidiliyor -> lat:{final_target_lat}, "
                f"lon:{final_target_lon}"
            )

        try:
            correction_north_m_s, correction_east_m_s, down_m_s, _, _ = (
                self.make_point_velocity_ned_command(
                    instant_target_latitude,
                    instant_target_longitude,
                    self.mission_altitude_,
                    self.nav_settings_.follower_xy_max_m_s,
                )
            )
            north_m_s = (
                self.nav_settings_.follower_feed_forward_gain * leader_ff_north_m_s
                + correction_north_m_s
            )
            east_m_s = (
                self.nav_settings_.follower_feed_forward_gain * leader_ff_east_m_s
                + correction_east_m_s
            )
            north_m_s, east_m_s = nav.limit_xy_velocity(
                north_m_s,
                east_m_s,
                self.nav_settings_.follower_xy_max_m_s,
            )
            await self.send_velocity_ned_with_offboard(
                north_m_s,
                east_m_s,
                down_m_s,
                track_heading,
            )
            self.move_command_sent_ = True
        except Exception as e:
            self.log.error(
                f"{self.role_str_} drone NED move komutu gonderilirken sorun meydana geldi: {e}"
            )
            self.move_command_sent_ = False
            await self.stop_ned_navigation()

    async def handle_takipci_goto_area(self):
        # Suruden ayrilan takipci, liderden paylasilan renkli alan koordinatina gider.
        if self.target_color_ is None:
            self.log.error("Alana gidis icin hedef renk belirlenmedi.")
            await self.stop_ned_navigation()
            return

        target_area = self.field_coordinates_.get(self.target_color_)
        if target_area is None:
            self.log.error(f"{self.target_color_} alan tipi tanimli degil.")
            await self.stop_ned_navigation()
            return

        target_area_lat = target_area["latitude"]
        target_area_lon = target_area["longitude"]
        if target_area_lat is None or target_area_lon is None:
            self.log.error(f"{self.target_color_} alan koordinatı henüz tanımlı değil.")
            await self.stop_ned_navigation()
            return

        if self.home_absolute_altitude_ is None:
            self.log.error("Alana gidis icin home absolute altitude henuz hazir degil.")
            await self.stop_ned_navigation()
            return

        current_lat = self.telemetry_info_["latitude"]
        current_lon = self.telemetry_info_["longitude"]
        current_heading = self.telemetry_info_["heading"]
        if current_lat is None or current_lon is None or current_heading is None:
            await self.set_idle_after_ned_stop("Alana gidis telemetri eksigi")
            return

        target_area_alt = (
            float(self.home_absolute_altitude_)
            + self.landing_settings_.area_approach_relative_altitude_m
        )

        try:
            target_reached = self.is_global_target_reached(
                target_area_lat,
                target_area_lon,
                target_area_alt,
                arrival_radius_m=self.nav_settings_.final_arrival_radius_m,
            )
        except ValueError:
            await self.stop_ned_navigation()
            return

        if target_reached:
            if not await self.confirm_ned_navigation_held("Renkli alana varis"):
                return
            self.is_departure_navigation_command_sent_ = False
            self.is_area_location_reached_ = True
            self.reset_land_to_area_session_state(clear_xy_error=True)
            self.log.info("Hedef noktaya varıldı.")
            return

        try:
            north_m_s, east_m_s, down_m_s, horizontal_error_m, _ = (
                self.make_point_velocity_ned_command(
                    target_area_lat,
                    target_area_lon,
                    target_area_alt,
                    self.nav_settings_.area_xy_max_m_s,
                )
            )
        except ValueError as e:
            self.log.error(f"Alana gidis icin NED hedef hatasi hesaplanamadi: {e}")
            await self.stop_ned_navigation()
            return

        target_heading = current_heading
        if horizontal_error_m > self.nav_settings_.arrival_radius_m:
            target_heading = nav.calculate_bearing(
                current_lat,
                current_lon,
                target_area_lat,
                target_area_lon,
                bearing_offset_deg=self.nav_settings_.bearing_offset_deg,
            )

        if not self.is_departure_navigation_command_sent_:
            self.log.info("Renkli alana NED gidiş emri verildi.")

        try:
            await self.send_velocity_ned_with_offboard(
                north_m_s,
                east_m_s,
                down_m_s,
                target_heading,
            )
            self.is_departure_navigation_command_sent_ = True
        except Exception as e:
            self.is_departure_navigation_command_sent_ = False
            self.log.error(
                f"Alana NED gidiş emri verilirken hata meydana geldi! Hata: {e}"
            )
            await self.stop_ned_navigation()

    async def handle_takipci_land_to_area(self):
        # Kamera merkezleme ve inis ayarlari.
        x_limit = 640.0
        y_limit = 360.0
        lambda_xy = 0.25
        max_speed = 1.0
        min_effective_altitude_m = 0.8

        relative_altitude = self.telemetry_info_["relative_altitude"]
        if relative_altitude is None:
            return

        now_s = time.monotonic()

        # Alan ustune geldikten sonra kisa sure sabit kalip goruntuyu oturt.
        if not self.land_to_area_stabilized_:
            if self.land_to_area_entry_time_ == 0.0:
                self.land_to_area_entry_time_ = now_s
                self.reset_x_y_error_state()
                self.log.info(
                    f"Renkli alan üstünde stabilizasyon bekleniyor "
                    f"({self.landing_settings_.stabilization_wait_s:.1f} s)."
                )

            if (
                now_s - self.land_to_area_entry_time_
            ) < self.landing_settings_.stabilization_wait_s:
                if await self.drone.offboard.is_active():
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                    )
                return

            self.land_to_area_stabilized_ = True
            self.log.info("Stabilizasyon tamamlandi, gorsel merkezleme baslatiliyor.")

        # Kamera piksel hatasini body-frame ileri/sag hizina cevir.
        norm_x = 999.0
        norm_y = 999.0
        forward_m_s = 0.0
        right_m_s = 0.0
        if self.x_y_error_["detected"]:
            norm_x = max(-1.0, min(self.x_y_error_["x"] / x_limit, 1.0))
            norm_y = max(-1.0, min(self.x_y_error_["y"] / y_limit, 1.0))
            gain_xy = lambda_xy * max(relative_altitude, min_effective_altitude_m)
            forward_m_s = max(-max_speed, min(-gain_xy * norm_y, max_speed))
            right_m_s = max(-max_speed, min(gain_xy * norm_x, max_speed))

        is_xy_aligned = (
            abs(norm_x) < (100.0 / x_limit)
            and abs(norm_y) < (100.0 / y_limit)
        )
        is_final_land_aligned = (
            abs(norm_x) < (self.landing_settings_.final_alignment_px / x_limit)
            and abs(norm_y) < (self.landing_settings_.final_alignment_px / y_limit)
        )

        if is_xy_aligned:
            self.last_xy_alignment_time_ = now_s
            self.landing_alignment_locked_ = True

        is_recently_aligned = (
            self.last_xy_alignment_time_ > 0.0
            and (now_s - self.last_xy_alignment_time_)
            <= self.landing_settings_.final_alignment_memory_s
        )

        # Native land komutu gonderildiyse inisin bitmesini bekle.
        if self.land_command_sent_:
            is_in_air = await anext(self.drone.telemetry.in_air().__aiter__())
            if not is_in_air:
                self.log.info("Drone renkli alana indi.")
                self.log.info("3 saniye bekleniyor...")
                await asyncio.sleep(3)

                self.is_landed_ = True
                self.is_swarm_departure_active = False
                self.is_area_location_reached_ = False
                self.is_departure_navigation_command_sent_ = False
                self.land_command_sent_ = False
                self.reset_land_to_area_session_state(clear_xy_error=True)
                self.publish_vision_control()

                self.is_arm_mission_done_ = False
                self.arm_command_sent_ = False

                self.is_takeoff_mission_done_ = False
                self.takeoff_command_sent_ = False

                self.is_initial_positioning_mission_done_ = False
                self.is_initial_positioning_command_sent_ = False

                self.state_ = DroneState.IDLE
            return

        # Alcakta ve hizaliysa offboard'dan native land moduna gec.
        if relative_altitude <= self.landing_settings_.final_transition_altitude_m:
            if is_final_land_aligned or is_recently_aligned:
                if await self.drone.offboard.is_active():
                    try:
                        await self.drone.offboard.stop()
                        self.nav_offboard_active_ = False
                    except Exception as e:
                        self.log.error(
                            f"final land aşamasında offboard kapatılamadı. Hata: {e}"
                        )
                await self.drone.action.land()
                self.land_command_sent_ = True
                self.log.info("Son yaklasim tamamlandi, native land moduna geciliyor.")
                return

        # Gorsel hedef kisa sure once kilitlendiyse cok alcakta kor inise izin ver.
        if relative_altitude <= self.landing_settings_.blind_transition_altitude_m:
            if self.landing_alignment_locked_:
                if await self.drone.offboard.is_active():
                    try:
                        await self.drone.offboard.stop()
                        self.nav_offboard_active_ = False
                    except Exception as e:
                        self.log.error(
                            f"final land aşamasında offboard kapatılamadı. Hata: {e}"
                        )
                await self.drone.action.land()
                self.land_command_sent_ = True
                self.log.info(
                    f"Gorsel hedef kaybolsa bile "
                    f"{self.landing_settings_.blind_transition_altitude_m:.1f} m altinda "
                    "native land moduna geciliyor."
                )
                return

        # Hedef merkezdeyse kontrollu alcal; degilse sadece yatay merkezleme yap.
        down_m_s = 0.0
        if is_xy_aligned:
            if relative_altitude > 2.0:
                down_m_s = 0.5
            elif relative_altitude > 1.0:
                down_m_s = 0.3
            else:
                down_m_s = 0.1

        if not await self.drone.offboard.is_active():
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await self.drone.offboard.start()

        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, 0.0)
        )

    def get_other_follower_initial_positioning_done(self):
        return all(
            bool(self.swarm_infos_[role_name]["initial_positioning_done"])
            for role_name in self.get_other_follower_role_keys()
        )

    def takipci_state_guncelle(self):
        leader_latitude = self.swarm_infos_["lider"]["latitude"]
        leader_longitude = self.swarm_infos_["lider"]["longitude"]
        formation_heading = self.get_leader_formation_heading_ref()
        other_follower_ready = self.get_other_follower_initial_positioning_done()

        # 1. Takipci once kendi kalkisini tamamlar.
        if not self.is_takeoff_mission_done_:
            self.state_ = DroneState.TAKE_OFF
            return True

        # 2. Final donus basladiysa takipci sadece RETURN_TO_HOME state'ine gecer.
        if self.is_return_to_home_after_mission_ and not self.is_all_mission_done_:
            self.state_ = DroneState.RETURN_TO_HOME
            return True

        # 3. Bu takipci suruden ayrildiysa once alana gider, sonra hassas inis yapar.
        if self.is_swarm_departure_active:
            if not self.is_area_location_reached_:
                self.reset_land_to_area_session_state()
                self.state_ = DroneState.GOTO_AREA
            elif not self.is_landed_:
                self.state_ = DroneState.LAND_TO_AREA
            return True

        # 4. Ilk formasyon slotuna gecmek icin lider konumu ve heading gerekir.
        if not self.is_initial_positioning_mission_done_:
            if (
                leader_latitude is None
                or leader_longitude is None
                or formation_heading is None
            ):
                self.log_once(
                    "liderin konum ve formasyon yönelimi bekleniyor...",
                    "rotasyonu_bekliyor",
                )
                self.state_ = DroneState.IDLE
            else:
                self.clear_logged_events("rotasyonu_bekliyor")
                self.state_ = DroneState.INITIAL_POSITIONING
            return True

        # 5. Lider rotasyon yayinliyorsa takipci de kendi slotunu gunceller.
        if (
            self.swarm_infos_["lider"]["rotation_active"]
            and not self.swarm_infos_["lider"]["rotating_done"]
        ):
            self.state_ = DroneState.ROTATION
            return True

        if self.nav_offboard_active_ and self.is_rotation_command_sent_:
            self.is_rotation_command_sent_ = False
            self.rotation_last_update_ = 0.0

        if self.is_all_mission_done_:
            self.log_once("tüm görevler tamamlandı. yeni görev bekleniyor...")
            self.state_ = DroneState.IDLE
            return True

        # 6. Lider rotasyonu bitmeden suru hareketine gecilmez.
        if not self.swarm_infos_["lider"]["rotating_done"]:
            self.rotation_alignment_done_ = False
            self.log_once(
                "ilk konumlanma tamamlandi, liderin yönelimini tamamlamasi bekleniyor...",
                "rotasyonu_bekliyor",
            )
            self.state_ = DroneState.IDLE
            return False

        self.clear_logged_events("rotasyonu_bekliyor")
        self.rotation_alignment_done_ = False

        # 7. Iki takipci de ilk konumunu almadan hareket baslamaz.
        if not other_follower_ready:
            self.log_once(
                f"drone rol: {self.role_str_} diğer takipçi drone'un yerini almasını bekliyor...",
                "yerini_almasini_bekliyor",
            )
            self.state_ = DroneState.IDLE
            return False

        self.clear_logged_events("yerini_almasini_bekliyor")

        # 8. Ortak hedefe gidilecekse MOVE state'i calisir.
        if not self.shared_target_reached_:
            self.state_ = DroneState.MOVE
            return True

        # 9. Final gorevde herkes ortak hedefe ulasinca return-home state'i baslar.
        if self.return_home_after_current_mission_:
            if not self.are_all_roles_shared_target_reached():
                self.log_once(
                    "Final gorevde tum drone'larin ortak hedef "
                    "formasyonuna ulasmasi bekleniyor..."
                )
                self.state_ = DroneState.IDLE
            else:
                self.is_return_to_home_after_mission_ = True
                self.state_ = DroneState.RETURN_TO_HOME
            return True

        self.state_ = DroneState.IDLE
        return True

    async def connect_drone(self):
        self.log.info("Drone bağlantısı bekleniyor...\n")

        connection_string = f"udpin://0.0.0.0:{self.own_udp_port_}"
        await self.drone.connect(connection_string)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.log.info("Drone bağlantısı başarılı!\n")
                self.is_connected_ = True
                break
            await asyncio.sleep(0.5)
            self.log.info("Drone bağlantısı bekleniyor...\n")

        self.log.info("Telemetri verilerinin gelmesi bekleniyor...\n")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.log.info("Global pozisyon ve ev konumu alındı.\n")
                break

        home = await anext(self.drone.telemetry.home().__aiter__())
        self.home_absolute_altitude_ = home.absolute_altitude_m
        self.home_latitude_ = getattr(home, "latitude_deg", None)
        self.home_longitude_ = getattr(home, "longitude_deg", None)

        self.log.info("Son telemetri verilerinin gelmesi bekleniyor...")
        asyncio.create_task(self.start_telemetry_collection())
        await asyncio.sleep(5)
        self.log.info("Telemetri verileri başarıyla alındı.")

        self.log.info("çarpışma kontrolü başlatılıyor...")
        try:
            asyncio.create_task(self.run_collision_control())
            await asyncio.sleep(1)
            self.log.info("Çarpışma kontrolü başarıyla başlatıldı.")
        except Exception as e:
            self.log.error(
                f"Drone: {self.role_str_} | Collision kontrol fonksiyonu başlatılırken hata meydana geldi: {e}"
            )

        self.log.info("Drone kalkışa hazır.\n")

    def get_formation_angles(self):
        if self.formation_type_ in (None, ""):
            return FORMATION_ANGLES[DEFAULT_FORMATION_TYPE]
        return FORMATION_ANGLES.get(
            self.formation_type_,
            FORMATION_ANGLES[DEFAULT_FORMATION_TYPE],
        )

    def get_role_relative_angle(self, role_name=None):
        role_name = role_name or self.role_str_
        side_key = self.get_role_side_key(role_name)
        return self.get_formation_angles().get(side_key)

    # Bu NED yardımcıları controller state'i okuduğu için burada kalır.
    # Saf matematik kısmı navigation_utils içindeki fonksiyonlara devredilmiştir.
    def get_current_target_error_ned_m(self, target_lat, target_lon, target_abs_alt):
        current_lat = self.telemetry_info_["latitude"]
        current_lon = self.telemetry_info_["longitude"]
        current_abs_alt = self.telemetry_info_["absolute_altitude"]
        if current_lat is None or current_lon is None or current_abs_alt is None:
            raise ValueError("NED hedef hatasi icin telemetri eksik")

        north_error_m, east_error_m = nav.global_error_to_ned_m(
            current_lat,
            current_lon,
            target_lat,
            target_lon,
        )
        down_error_m = float(current_abs_alt) - float(target_abs_alt)
        return north_error_m, east_error_m, down_error_m

    def is_global_target_reached(
        self,
        target_lat,
        target_lon,
        target_abs_alt,
        arrival_radius_m=None,
        alt_tol_m=None,
    ):
        arrival_radius_m = (
            self.nav_settings_.arrival_radius_m
            if arrival_radius_m is None
            else float(arrival_radius_m)
        )
        alt_tol_m = (
            self.nav_settings_.alt_tolerance_m
            if alt_tol_m is None
            else float(alt_tol_m)
        )
        north_error_m, east_error_m, down_error_m = self.get_current_target_error_ned_m(
            target_lat,
            target_lon,
            target_abs_alt,
        )

        return (
            math.hypot(north_error_m, east_error_m) <= arrival_radius_m
            and abs(down_error_m) <= alt_tol_m
        )

    def make_point_velocity_ned_command(
        self,
        target_lat,
        target_lon,
        target_abs_alt,
        max_xy_m_s,
        kp_xy=None,
        kp_z=None,
        slow_radius_m=None,
    ):
        kp_xy = self.nav_settings_.kp_xy if kp_xy is None else float(kp_xy)
        kp_z = self.nav_settings_.kp_z if kp_z is None else float(kp_z)
        slow_radius_m = (
            self.nav_settings_.slow_radius_m
            if slow_radius_m is None
            else float(slow_radius_m)
        )

        north_error_m, east_error_m, down_error_m = self.get_current_target_error_ned_m(
            target_lat,
            target_lon,
            target_abs_alt,
        )
        north_m_s, east_m_s, horizontal_error_m = nav.make_xy_velocity_from_error(
            north_error_m,
            east_error_m,
            max_xy_m_s,
            kp_xy,
            slow_radius_m,
            self.nav_settings_.hold_speed_epsilon_m_s,
        )
        down_m_s = kp_z * down_error_m
        max_down_m_s = abs(float(self.nav_settings_.vertical_max_m_s))
        down_m_s = max(-max_down_m_s, min(float(down_m_s), max_down_m_s))
        return north_m_s, east_m_s, down_m_s, horizontal_error_m, down_error_m

    def reset_ned_velocity_limiter(self):
        self.last_ned_velocity_cmd_ = (0.0, 0.0, 0.0)
        self.last_ned_velocity_cmd_time_s_ = None

    def limit_ned_velocity_acceleration(self, north_m_s, east_m_s, down_m_s):
        now_s = time.monotonic()
        last_north_m_s, last_east_m_s, last_down_m_s = self.last_ned_velocity_cmd_
        if self.last_ned_velocity_cmd_time_s_ is None:
            dt_s = self.nav_settings_.command_period_s
        else:
            dt_s = max(now_s - self.last_ned_velocity_cmd_time_s_, 0.001)

        north_delta_m_s = float(north_m_s) - last_north_m_s
        east_delta_m_s = float(east_m_s) - last_east_m_s
        max_xy_delta_m_s = self.nav_settings_.xy_accel_limit_m_s2 * dt_s
        xy_delta_m_s = math.hypot(north_delta_m_s, east_delta_m_s)
        if xy_delta_m_s > max_xy_delta_m_s > 0.0:
            scale = max_xy_delta_m_s / xy_delta_m_s
            north_m_s = last_north_m_s + north_delta_m_s * scale
            east_m_s = last_east_m_s + east_delta_m_s * scale

        max_down_delta_m_s = self.nav_settings_.vertical_accel_limit_m_s2 * dt_s
        down_delta_m_s = float(down_m_s) - last_down_m_s
        down_delta_m_s = max(
            -max_down_delta_m_s,
            min(down_delta_m_s, max_down_delta_m_s),
        )
        down_m_s = last_down_m_s + down_delta_m_s

        return float(north_m_s), float(east_m_s), float(down_m_s)

    def remember_ned_velocity_command(self, north_m_s, east_m_s, down_m_s):
        self.last_ned_velocity_cmd_ = (
            float(north_m_s),
            float(east_m_s),
            float(down_m_s),
        )
        self.last_ned_velocity_cmd_time_s_ = time.monotonic()

    async def send_velocity_ned(self, north_m_s, east_m_s, down_m_s, yaw_deg):
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                float(north_m_s),
                float(east_m_s),
                float(down_m_s),
                float(yaw_deg),
            )
        )

    async def send_velocity_ned_with_offboard(
        self,
        north_m_s,
        east_m_s,
        down_m_s,
        yaw_deg,
    ):
        await self.ensure_ned_offboard_started(
            yaw_deg,
            north_m_s=north_m_s,
            east_m_s=east_m_s,
            down_m_s=down_m_s,
        )
        north_m_s, east_m_s, down_m_s = self.limit_ned_velocity_acceleration(
            north_m_s,
            east_m_s,
            down_m_s,
        )
        await self.send_velocity_ned(north_m_s, east_m_s, down_m_s, yaw_deg)
        self.remember_ned_velocity_command(north_m_s, east_m_s, down_m_s)

    async def ensure_ned_offboard_started(
        self,
        yaw_deg,
        north_m_s=0.0,
        east_m_s=0.0,
        down_m_s=0.0,
    ):
        if await self.drone.offboard.is_active():
            self.nav_offboard_active_ = True
            return

        for _ in range(self.nav_settings_.offboard_prestream_count):
            await self.send_velocity_ned(
                0.0,
                0.0,
                0.0,
                yaw_deg,
            )
            await asyncio.sleep(self.nav_settings_.command_period_s)

        try:
            await self.drone.offboard.start()
            self.nav_offboard_active_ = True
            self.reset_ned_velocity_limiter()
        except OffboardError:
            self.nav_offboard_active_ = False
            raise

    def get_hold_yaw_deg(self, fallback_yaw_deg=None):
        if fallback_yaw_deg is not None:
            return float(fallback_yaw_deg)

        current_heading = self.telemetry_info_.get("heading")
        if current_heading is not None:
            return float(current_heading)

        if self.formation_heading_ref_ is not None:
            return float(self.formation_heading_ref_)

        leader_heading = self.swarm_infos_.get("lider", {}).get("heading")
        if leader_heading is not None:
            return float(leader_heading)

        return 0.0

    async def hold_ned_navigation(self, yaw_deg=None, start_if_needed=False):
        hold_yaw_deg = self.get_hold_yaw_deg(yaw_deg)

        try:
            offboard_active = await self.drone.offboard.is_active()
        except Exception as e:
            self.nav_offboard_active_ = True
            self.log.error(f"NED hold icin offboard durumu okunamadi: {e}")
            return False

        if not offboard_active:
            self.nav_offboard_active_ = False
            if not start_if_needed:
                self.reset_ned_velocity_limiter()
                return True
            try:
                await self.ensure_ned_offboard_started(hold_yaw_deg)
            except Exception as e:
                self.log.error(f"NED hold icin offboard baslatilamadi: {e}")
                return False

        try:
            await self.ensure_ned_offboard_started(hold_yaw_deg)
            await self.send_velocity_ned(
                0.0,
                0.0,
                0.0,
                hold_yaw_deg,
            )
            self.remember_ned_velocity_command(0.0, 0.0, 0.0)
            self.nav_offboard_active_ = True
            return True
        except Exception as e:
            self.log.error(f"NED hold komutu gonderilemedi: {e}")
            self.nav_offboard_active_ = False
            return False

    async def hold_ned_navigation_for(self, duration_s, yaw_deg=None):
        deadline_s = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline_s:
            if not await self.hold_ned_navigation(yaw_deg, start_if_needed=True):
                return False
            await asyncio.sleep(self.nav_settings_.command_period_s)
        return await self.hold_ned_navigation(yaw_deg, start_if_needed=True)

    async def stop_ned_navigation(self):
        try:
            offboard_active = await self.drone.offboard.is_active()
        except Exception as e:
            self.nav_offboard_active_ = True
            self.log.error(f"NED offboard durumu okunamadi: {e}")
            return False

        if not offboard_active:
            self.nav_offboard_active_ = False
            self.reset_ned_velocity_limiter()
            return True

        try:
            hold_yaw_deg = self.get_hold_yaw_deg()
            for _ in range(self.nav_settings_.offboard_prestream_count):
                north_m_s, east_m_s, down_m_s = self.limit_ned_velocity_acceleration(
                    0.0,
                    0.0,
                    0.0,
                )
                await self.send_velocity_ned(
                    north_m_s,
                    east_m_s,
                    down_m_s,
                    hold_yaw_deg,
                )
                self.remember_ned_velocity_command(
                    north_m_s,
                    east_m_s,
                    down_m_s,
                )
                if math.hypot(north_m_s, east_m_s) <= 0.05 and abs(down_m_s) <= 0.05:
                    break
                await asyncio.sleep(self.nav_settings_.command_period_s)
            await self.drone.offboard.stop()
            self.nav_offboard_active_ = False
            self.reset_ned_velocity_limiter()
            return True
        except OffboardError as e:
            self.nav_offboard_active_ = True
            self.log.error(f"NED offboard kapatilamadi: {e}")
            return False
        except Exception as e:
            self.nav_offboard_active_ = True
            self.log.error(f"NED offboard kapatilirken beklenmeyen hata: {e}")
            return False

    async def confirm_ned_navigation_stopped(self, reason):
        if await self.stop_ned_navigation():
            return True

        self.log.error(
            f"{reason}: NED offboard kapatilamadigi icin durum gecisi ertelendi."
        )
        return False

    async def confirm_ned_navigation_held(self, reason):
        if await self.hold_ned_navigation_for(0.5):
            return True

        self.log.error(
            f"{reason}: NED hold komutu gonderilemedigi icin durum gecisi ertelendi."
        )
        return False

    async def set_idle_after_ned_stop(self, reason):
        if not await self.confirm_ned_navigation_stopped(reason):
            return False

        self.state_ = DroneState.IDLE
        return True

    def resolve_leader_target_reference(self, reference_heading_deg=None):
        leader_target_lat = self.target_positions_["lider"]["latitude"]
        leader_target_lon = self.target_positions_["lider"]["longitude"]
        leader_heading = reference_heading_deg

        if leader_heading is None:
            leader_lat = self.swarm_infos_["lider"]["latitude"]
            leader_lon = self.swarm_infos_["lider"]["longitude"]
            if (
                leader_lat is not None
                and leader_lon is not None
                and leader_target_lat is not None
                and leader_target_lon is not None
            ):
                leader_heading = nav.calculate_bearing(
                    float(leader_lat),
                    float(leader_lon),
                    float(leader_target_lat),
                    float(leader_target_lon),
                    bearing_offset_deg=self.nav_settings_.bearing_offset_deg,
                )
            else:
                leader_heading = self.swarm_infos_["lider"]["heading"]

        if leader_target_lat is None or leader_target_lon is None or leader_heading is None:
            raise ValueError("lider hedefi veya referans heading hesaplanamadi")

        return leader_target_lat, leader_target_lon, leader_heading

    def get_follower_relative_angles(self):
        relative_angles = {}
        for role_name in self.get_follower_role_keys():
            relative_angle = self.get_role_relative_angle(role_name)
            if relative_angle is None:
                raise ValueError(f"{role_name} relatif acisi hesaplanamadi")
            relative_angles[role_name] = relative_angle
        return relative_angles

    def update_target_positions(self, reference_heading_deg=None):
        leader_target_lat, leader_target_lon, leader_heading = (
            self.resolve_leader_target_reference(reference_heading_deg)
        )
        relative_angles = self.get_follower_relative_angles()

        updated_positions = {}
        for role_name, relative_angle in relative_angles.items():
            updated_positions[role_name] = nav.offset_coordinate(
                leader_target_lat,
                leader_target_lon,
                leader_heading,
                relative_angle,
                self.get_formation_horizontal_distance_m(role_name),
            )

        for role_name, (target_lat, target_lon) in updated_positions.items():
            self.target_positions_[role_name]["latitude"] = target_lat
            self.target_positions_[role_name]["longitude"] = target_lon

    def clear_swarm_target_positions(self):
        for role in self.target_positions_:
            self.target_positions_[role]["latitude"] = None
            self.target_positions_[role]["longitude"] = None

    def resolve_shared_home_target(self):
        if self.role_ == DroneRole.LEADER:
            return (
                self.home_latitude_,
                self.home_longitude_,
                self.home_absolute_altitude_,
            )

        return (
            self.leader_home_latitude_,
            self.leader_home_longitude_,
            self.leader_home_absolute_altitude_,
        )

    def set_final_return_home_target_if_needed(self):
        if not self.return_home_after_current_mission_:
            return False

        # Final gorevde ortak hedef, liderin yayinladigi home koordinatidir.
        shared_target_lat, shared_target_lon, _ = self.resolve_shared_home_target()
        if shared_target_lat is None or shared_target_lon is None:
            self.clear_swarm_target_positions()
            self.log.error("Final görev için liderin kalkış koordinatı henüz alınmadı.")
            return True

        self.target_positions_["lider"]["latitude"] = float(shared_target_lat)
        self.target_positions_["lider"]["longitude"] = float(shared_target_lon)
        return True

    def resolve_return_home_target_altitude(self, target_home_abs_alt):
        # Eve donerken ani alcak ucustan kacinmak icin mevcut guvenli irtifalarin
        # en yuksegi secilir.
        candidates = []
        if self.mission_altitude_ is not None:
            candidates.append(float(self.mission_altitude_))
        if self.telemetry_info_["absolute_altitude"] is not None:
            candidates.append(float(self.telemetry_info_["absolute_altitude"]))
        if target_home_abs_alt is not None and self.takeoff_altitude_ is not None:
            candidates.append(float(target_home_abs_alt) + float(self.takeoff_altitude_))
        if not candidates:
            return None
        return max(candidates)

    def resolve_return_home_heading(
        self,
        current_lat,
        current_lon,
        target_lat,
        target_lon,
        horizontal_error_m,
    ):
        current_heading = self.telemetry_info_["heading"]
        target_heading = current_heading if current_heading is not None else 0.0
        if horizontal_error_m > self.nav_settings_.final_arrival_radius_m:
            target_heading = nav.calculate_bearing(
                current_lat,
                current_lon,
                target_lat,
                target_lon,
                bearing_offset_deg=self.nav_settings_.bearing_offset_deg,
            )
        return target_heading

    def mark_return_home_completed(self):
        self.return_home_after_current_mission_ = False
        self.shared_target_reached_ = False
        self.reset_return_home_sequence()
        self.is_all_mission_done_ = True
        self.state_ = DroneState.IDLE

    async def run_return_to_home(self):
        # 1. Home koordinati ve mevcut konum hazir degilse hover/stop durumunda kal.
        target_lat = self.home_latitude_
        target_lon = self.home_longitude_
        target_home_abs_alt = self.home_absolute_altitude_
        if target_lat is None or target_lon is None:
            self.log_once(OWN_HOME_WAIT_MESSAGE)
            await self.stop_ned_navigation()
            return

        self.clear_logged_events(OWN_HOME_WAIT_MESSAGE)

        current_lat = self.telemetry_info_["latitude"]
        current_lon = self.telemetry_info_["longitude"]
        if current_lat is None or current_lon is None:
            await self.stop_ned_navigation()
            return

        # 2. Donus boyunca kullanilacak mutlak irtifayi belirle.
        target_abs_alt = self.resolve_return_home_target_altitude(
            target_home_abs_alt,
        )
        if target_abs_alt is None:
            await self.stop_ned_navigation()
            return

        # 3. Inis komutu gonderilmediyse NED velocity ile home noktasina ilerle.
        if not self.return_home_land_command_sent_:
            try:
                north_m_s, east_m_s, down_m_s, horizontal_error_m, _ = (
                    self.make_point_velocity_ned_command(
                        target_lat,
                        target_lon,
                        target_abs_alt,
                        self.nav_settings_.return_xy_max_m_s,
                    )
                )
            except ValueError as e:
                self.log.error(f"Eve donus icin NED hedef hatasi hesaplanamadi: {e}")
                await self.stop_ned_navigation()
                return

            target_heading = self.resolve_return_home_heading(
                current_lat,
                current_lon,
                target_lat,
                target_lon,
                horizontal_error_m,
            )

            try:
                await self.send_velocity_ned_with_offboard(
                    north_m_s,
                    east_m_s,
                    down_m_s,
                    target_heading,
                )
                self.log_once(
                    "Final gorev sonrasi kendi home noktasina NED donus baslatildi -> "
                    f"lat:{float(target_lat)}, lon:{float(target_lon)}",
                    RETURN_HOME_STARTED_EVENT_KEY,
                )
            except Exception as e:
                self.log.error(f"Eve donus NED komutu hatasi: {e}")
                await self.stop_ned_navigation()
                return

            # 4. Home hedefine varildiyse offboard'u durdurup normal inis baslat.
            try:
                target_reached = self.is_global_target_reached(
                    target_lat,
                    target_lon,
                    target_abs_alt,
                    arrival_radius_m=RETURN_HOME_ARRIVAL_RADIUS_M,
                )
            except ValueError:
                await self.stop_ned_navigation()
                return
            if not target_reached:
                return

            if not await self.confirm_ned_navigation_stopped(
                "Eve donus hedefe varis sonrasi inis"
            ):
                return
            try:
                await self.drone.action.land()
            except ActionError as e:
                self.log.error(
                    f"Ortak home noktasinda inis komutu hatasi: {e._result_str}"
                )
                return

            self.return_home_land_command_sent_ = True
            self.log.info("Kendi home noktasina varildi, inis baslatildi.")
            return

        # 5. Inis komutu gonderildikten sonra MAVSDK in_air bilgisi ile bitisi bekle.
        is_in_air = await anext(self.drone.telemetry.in_air().__aiter__())
        if is_in_air:
            return

        self.mark_return_home_completed()
        self.log.info("Drone kendi home noktasina indi. Gorev tamamlandi.")


def main(args=None):
    rclpy.init(args=args)

    drone_controller_node = DroneControllerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(drone_controller_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
