import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class ROS2MQTTPublisher(Node):
    def __init__(self):
        super().__init__('ros2_mqtt_publisher')

        # MQTT Config
        self.broker = "test.mosquitto.org"
        self.port = 1883
        self.robot_name = "fibotx1"
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.connect(self.broker, self.port, 60)

        # ROS 2 Subscribers
        self.subscribers = {}
        for joint in range(1, 7):
            topic_name = f"{self.robot_name}/joint{joint}"
            self.subscribers[topic_name] = self.create_subscription(
                String, topic_name, self.ros_callback, 10)

    def ros_callback(self, msg, topic):
        self.get_logger().info(f"📤 Sending to MQTT: {topic} -> {msg.data}")
        self.mqtt_client.publish(topic, msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ROS2MQTTPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
