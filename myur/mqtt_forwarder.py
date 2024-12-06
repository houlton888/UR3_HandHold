import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


class MqttToRos2Node(Node):
    def __init__(self, mqtt_topic, ros2_topic, host, port):
        super().__init__("mqtt_to_ros2_node")

        # ROS2 Publisher
        self.ros2_topic = ros2_topic
        self.publisher_ = self.create_publisher(String, ros2_topic, 10)

        # MQTT Client
        self.mqtt_client = mqtt.Client()
        self.mqtt_topic = mqtt_topic

        # MQTT Callbacks
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to the MQTT Broker
        self.mqtt_client.connect(host, port, 60)

        # Start the MQTT client loop
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(
                f"Connected to MQTT Broker! Subscribing to topic: {self.mqtt_topic}"
            )
            self.mqtt_client.subscribe(self.mqtt_topic)
        else:
            self.get_logger().error(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        try:
            ros2_msg = String()
            ros2_msg.data = msg.payload.decode("utf-8")
            self.publisher_.publish(ros2_msg)
            self.get_logger().info(
                f"Forwarded MQTT message to ROS2 topic {self.ros2_topic}: {ros2_msg.data}"
            )
        except Exception as e:
            self.get_logger().error(f"Error processing message: {str(e)}")


import argparse

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Create an MQTT forwarder.")
    parser.add_argument("-t", type=str, required=True, help="MQTT topic name")
    parser.add_argument("-h", type=str, required=True, help="MQTT broker host")
    parser.add_argument("-p", type=str, required=True, help="MQTT broker port")
    args = parser.parse_args()

    rclpy.init()

    mqtt_topic = args.t
    ros2_topic = "mqtt/"+mqtt_topic

    node = MqttToRos2Node(mqtt_topic, ros2_topic, args.h, args.p)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
