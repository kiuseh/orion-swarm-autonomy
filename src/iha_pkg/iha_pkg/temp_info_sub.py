import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Temp_info_subscriber(Node):
    def __init__(self):
        super().__init__("temp_subscriber_node")
        self.log = self.get_logger()

        self.temp_info_subscriber_ = self.create_subscription(String, "temp_topic", self.temp_info_subscriber, 10)

        self.log.info("temp_subscriber_node başlatıldı.")

    def temp_info_subscriber(self, msg):
        self.log.info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = Temp_info_subscriber()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
