import json
import sys
import threading

import rclpy
from PySide6.QtWidgets import QApplication
from rclpy.node import Node
from std_msgs.msg import String

from arayuz_pkg.arayuz import DroneArayuzu


class MyNode(Node):
    def __init__(self):
        super().__init__("python_test")
        self.command_publisher_ = self.create_publisher(String, "arayuz_topic", 10)
        self.emergency_land_publisher_ = self.create_publisher(
            String,
            "emergency_land_topic",
            10,
        )

        self.ui = DroneArayuzu()
        self.ui.kalkis_buton.clicked.connect(self.kalkis_action)
        self.ui.acil_inis_buton.clicked.connect(self.acil_inis_action)
        self.ui.show()

    def kalkis_action(self):
        latitude_text = self.ui.latitude_input.text().strip()
        longitude_text = self.ui.longitude_input.text().strip()
        takeoff_altitude_text = self.ui.takeoff_altitude_input.text().strip()
        follow_distance_text = self.ui.follow_distance_input.text().strip()

        if (
            not latitude_text
            or not longitude_text
            or not takeoff_altitude_text
            or not follow_distance_text
        ):
            self.get_logger().error(
                "latitude, longitude, takeoff altitude ve drone'lar arası mesafe "
                "alanları boş bırakılamaz."
            )
            return

        try:
            latitude = float(latitude_text)
            longitude = float(longitude_text)
            takeoff_altitude = float(takeoff_altitude_text)
            follow_distance = float(follow_distance_text)
        except ValueError:
            self.get_logger().error(
                "latitude, longitude, takeoff altitude ve drone'lar arası mesafe "
                "değerleri sayı formatında olmalı."
            )
            return

        if takeoff_altitude <= 0:
            self.get_logger().error("takeoff altitude 0'dan büyük olmalı.")
            return
        if follow_distance <= 0:
            self.get_logger().error("drone'lar arası mesafe 0'dan büyük olmalı.")
            return

        plan = {
            "roles": {
                14541: -1,
                14542: 0,
                14543: 1,
            },
            "takeoff_altitude": takeoff_altitude,
            "follow_distance_m": follow_distance,
            "latitude": latitude,
            "longitude": longitude,
        }

        msg = String()
        msg.data = json.dumps(plan)
        self.command_publisher_.publish(msg)
        self.get_logger().info(
            f"kalkış planı yayınlandı. ilk hedef lat={latitude}, lon={longitude}, "
            f"takeoff={takeoff_altitude}m, mesafe={follow_distance}m"
        )

    def acil_inis_action(self):
        msg = String()
        msg.data = json.dumps({"command": "emergency_land"})
        self.emergency_land_publisher_.publish(msg)
        self.get_logger().warn("acil iniş komutu yayınlandı.")


def main(args=None):
    app = QApplication(sys.argv)
    rclpy.init(args=args)
    node = MyNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    app.exec()

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
