# myur/launch_ur.py

import subprocess
import argparse
import os

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Launch UR control")
    parser.add_argument('--ur_type', type=str, default='ur3e', help='Type of UR robot')
    parser.add_argument('--robot_ip', type=str, default="130.64.17.95", help='IP address of the robot')
    parser.add_argument('--launch_rviz', type=bool, default=False, help='Launch RViz')
    parser.add_argument('--controller', type=str, default='forward_velocity_controller', help='Initial Controller')
    
    args = parser.parse_args()
    
    # Build the command with arguments
    # ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=130.64.17.5 launch_rviz:=false
    command = [
        "ros2", "launch", "ur_robot_driver", "ur3e.launch.py",
        f"ur_type:={args.ur_type}", f"robot_ip:={args.robot_ip}", f"launch_rviz:={str(args.launch_rviz).lower()}", 
        f"initial_joint_controller:={args.controller}"
    ]

    # Execute the command
    subprocess.run(command)

if __name__ == "__main__":
    main()