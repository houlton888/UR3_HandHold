import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from myur import robotiq_gripper
import time

class Gripper(Node):

    def __init__(self,IP,calibrate=True):
        # initialize the topic (name it, create the publisher and publishing rate
        super().__init__("GripperNode")
        print("Initizializing Gripper Node")
        self.get_logger().info("Initizializing Gripper Node")
        self.publisher_ = self.create_publisher(Int32MultiArray, "/gripper/state", 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            Int32MultiArray, "/gripper/control", self.listener_callback, 10
        )

        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(IP, 63352)
        self.gripper.activate(auto_calibrate=calibrate)

        self.target = -1

    def timer_callback(self):
        # every interval, create and publish gripper state
        msg = Int32MultiArray()
        msg.data = [
            int(self.gripper.get_current_position()),
            int(self.gripper.get_current_speed()),
            int(self.gripper.get_current_force()),
            int(self.target),
            int(self.gripper._get_var(self.gripper.PRE)),
            int(self.gripper._get_var(self.gripper.OBJ))
        ]
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        try:
            info = f"Gripper Node Received Control: {list(msg.data)}"
            self.get_logger().info(info)
            print(info)
            [POS, SPE, FOR] = msg.data
            null, self.target = self.gripper.move(POS, SPE, FOR)
        except Exception as e:
            self.get_logger().info(f"Incorrect Gripper Control Format: [POS,SPE,FOR]")

import argparse

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Start the Gripper Node")
    parser.add_argument('-sc', action='store_true', help='Skip calibration')
    parser.add_argument('--ip', type=str, required=True, help='IP address of the gripper')

    args = parser.parse_args()

    rclpy.init()

    try:
        if args.sc:
            print("Skipping calibration...")
            gripper_node = Gripper(args.ip,calibrate=False)
        else:
            print("Performing calibration...")
            gripper_node = Gripper(args.ip,calibrate=True)
    except Exception as e:
        print("Gripper node failed: Check that robot power is on.")

    print("Gripper Node started")
    rclpy.spin(gripper_node)  # The code stays here forever
    gripper_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
