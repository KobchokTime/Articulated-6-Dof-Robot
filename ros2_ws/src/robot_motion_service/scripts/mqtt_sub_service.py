#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray 
import paho.mqtt.client as mqtt

class MQTTROS2Bridge(Node):
    def __init__(self):
        super().__init__('mqtt_ros2_subscriber')

        # MQTT Configuration
        self.broker = "test.mosquitto.org"
        self.port = 1883
        self.robot_name = "fibotx1"
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.connect(self.broker, self.port, 60)
#!/usr/bin/env python3

        # ROS 2 Publisher for velocity controller commands
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # MQTT Subscriber setup
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.get_logger().info("✅ Connected to MQTT Broker")
            for joint in range(1, 7):  # Subscribe to joint1 to joint6
                topic = f"{self.robot_name}/joint{joint}"
                client.subscribe(topic)
                self.get_logger().info(f"🔔 Subscribed to {topic}")
        else:
            self.get_logger().info(f"❌ Failed to connect, return code {rc}")

    def on_message(client, userdata, msg):
    	try:
	    value = float(msg.payload.decode())  # แปลง string เป็น float
	    joint_name = msg.topic.split("/")[-1]  # ดึงชื่อ joint เช่น "joint1"

	    # ส่งค่าไปยัง ROS2
	    msg_to_ros = Float64MultiArray()
	    msg_to_ros.data = [0.0] * 6  # กำหนดค่าเริ่มต้นเป็น 0 ทั้ง 6 joint
	
    	    # หาตำแหน่ง joint (joint1 -> index 0, joint2 -> index 1, ...)
	    joint_index = int(joint_name.replace("joint", "")) - 1
	    msg_to_ros.data[joint_index] = value  # อัปเดตค่าของ joint ที่ต้องการ
	
	    publisher.publish(msg_to_ros)
	    node.get_logger().info(f"📤 Published to ROS: {msg_to_ros.data}")

        except ValueError:
	    node.get_logger().error(f"❌ Error processing message: {msg.payload.decode()} is not a valid float")


 """   def on_message(self, client, userdata, msg):
        topic = msg.topic
        message = msg.payload.decode()  # Assuming message is a string

        # Log the received message
        self.get_logger().info(f"📥 Received from {topic}: {message}")

        # Split the message into values for x, y, z, roll, pitch, yaw
        try:
            values = list(map(float, message.split(',')))  # Convert message to list of floats

            if len(values) == 6:  # Ensure there are exactly 6 values
                # Create Float64MultiArray message
                velocity_msg = Float64MultiArray()
                velocity_msg.data = values  # Set the data of the array to the received values

                # Publish the message to /velocity_controller/commands
                self.velocity_publisher.publish(velocity_msg)
                self.get_logger().info(f"📤 Published to /velocity_controller/commands: {velocity_msg.data}")
            else:
                self.get_logger().error(f"❌ Invalid number of values received: {len(values)}")
        except Exception as e:
            self.get_logger().error(f"❌ Error processing message: {str(e)}")
"""

def main(args=None):
    rclpy.init(args=args)
    node = MQTTROS2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
