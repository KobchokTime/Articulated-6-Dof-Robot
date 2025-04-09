import paho.mqtt.client as mqtt
import time

broker = "test.mosquitto.org"
port = 1883

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(broker, port, 60)

robots = ["fibotx1", "fibotx2"]
joints = [1, 2, 3, 4, 5, 6]
angle = [ 22.44,33.09,7.89,-79.81,9.09,27.45]
for robot in robots:
    for i in range(0,7):
        topic = f"{robot}/joint{joints[i]}"
        message = f"Moving {topic}"
        client.publish(topic, message)
        print(f"🚀 Published to {topic}: {message}")
        time.sleep(1)

client.disconnect()
