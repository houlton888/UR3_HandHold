import rclpy
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
import math
import time
# ROS2 Msg Types:
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
# Internal Libraries:
from myur.ik_solver.ur_kinematics import URKinematics
#New Inverse solver libraries:
import roboticstoolbox as rtb
import numpy as np

#EVERYTHING IS IN RADIANS

class Arm_Velocity(Node):
    def __init__(self, pub_freq = .1):
        rclpy.init()
        super().__init__('remote_control_client_velo')
         # Action Client Setup
        self.declare_parameter("controller_name", "forward_velocity_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        controller_name = (
            self.get_parameter("controller_name").value + "/forward_velocity_controller"
        )
        self.joints = self.get_parameter("joints").value
        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')
            
        
        # Set up publisher to the joint velocity command topic
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            10
        )
        
        # Define the velocity command (in radians per second for each joint)
        self.frequency = pub_freq
        self.velocity_command = Float64MultiArray()
        self.velocity_command.data = ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Timer to continuously publish the current velocity command at 10 Hz
        self.timer = self.create_timer(self.frequency, self.publish_velocity_command)
        
        # Public Attributes
        self.ik_solver = URKinematics("ur3e")
        self.ik_arm = rtb.models.UR3()
        self.joint_states = JointStates()
        self.tool_wrench = ToolWrench()
        
    def __del__(self):
        # De-init rclpy
        print("deleting reached")
        rclpy.shutdown()

    def set_vel(self, velocities):
        """
        Update the velocity command with a list of joint velocities.
        :param velocities: List of floats representing joint velocities.
        """
        self.velocity_command.data = velocities
        self.publish_velocity_command()
        rclpy.spin_once(self)
        
    def publish_velocity_command(self):
        """Publish the current velocity command."""
        # Publish the velocity command
        self.publisher.publish(self.velocity_command)
        self.get_logger().info(f"Sent joint velocities: {self.velocity_command.data}")
        
    def read_joints_pos(self):
        """
        Get the angle of each joint in radians.

        """
        return self.joint_states.get_joints()["position"]

    def read_global_pos(self,euler=True):
        """
        Get the global position of end effector in meters.

        Returns:
            list: [x,y,z,rx,ry,rz]
        """
        return self.solve_fk(self.read_joints_pos(), euler)
        
    def solve_ik(self, cords, q_guess=None):
        """
        HAS ISSUE WHERE FAILS SOMETIMES:
        Solve inverse kinematics for given coordinates. 

        Args:
            cords (list): A list of coordinates, either [x, y, z, rx, ry, rz] or [x, y, z, qw, qx, qy, qz].
            q_guess (list, optional): A list of joint angles (in radians) used to find the closest IK solution in radians.
        Returns:
            list: Joint positions in radians that achieve the given coordinates. [see self.joints]
        """
        if q_guess is None:  # Use current robot pose as q_guess
            q_guess = self.read_joints_pos()


        # If coordinates in euler format convert to quaternions
        if len(cords) == 6:
            for i, angle in enumerate(
                cords[3:7]
            ):  # Deviate zeros to prevent unsolved ik
                if angle == 0:
                    cords[i + 3] = 0.1
            r = R.from_euler("zyx", cords[3:6], degrees=False)
            quat = r.as_quat(scalar_first=True).tolist()
            cords = cords[0:3] + quat
            
        ik_angles = self.ik_solver.inverse(cords, False, q_guess=q_guess)
        if ik_angles is None:
            for i in range(5):
                if ik_angles is None:
                    print("attempting solve again iteration number " + str(i))
                    ik_angles = self.ik_solver.inverse(cords, False, q_guess=self.read_joints_pos())
                    if i == 4:
                        self.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                        raise ValueError("solve_ik could not find inverse solve after 5 attmpts (line 123)")
        return ik_angles  # output is radians

    def solve_ik_velo(self, xyz_vel):
        """
        From library: https://github.com/jhavl/dkt/blob/main/Part%201/3%20Resolved-Rate%20Motion%20Control.ipynb
        input: xyz_vel -> velocity wanted in xyz cords:
        returns: joint velocities
        """
        #print(self.ik_arm.ee_links[0])
        #print(xyz_vel)
        velo = np.append(xyz_vel,[0.0, 0.0, 0.0])
        #print("line 145 vel: " +str(velo))
        # The base-frame manipulator Jacobian in the qr configuration
        current_position = self.read_joints_pos() # in radians  =r.read_joints_pos()
        J = self.ik_arm.jacob0(current_position)
        
        # Calculate the required joint velocities to achieve the desired end-effector velocity ev
        dq = np.linalg.pinv(J) @ velo
        
        # return results
        return np.round(dq, 6) #6 is num points to round to


    def solve_fk(self, angles, euler=True):
        """
        Solve forward kinematics for given joint positions.

        Args:
            angles (list): A list of joint angles.
            degrees (bool): True if degrees, False if radians.
            euler (bool): True if euler rotation desired, False for quaternion.
        Returns:
            list: End effector coordinates resulting from joint angles.
        """
        cords_q = self.ik_solver.forward(angles)
        if euler:
            r = R.from_quat(cords_q[3:7], scalar_first=True)
            euler = r.as_euler("zyx", degrees=False).tolist()
            cords = cords_q[0:3].tolist() + euler
        else:
            cords = cords_q

        return cords
        
    def get_force(self):
        """
        Get the force exerted on the end effector in Newtons (relative to the end effector).

        Returns:
            list: [x,y,z]
        """
        return self.tool_wrench.get()["force"]

    def get_torque(self):
        """
        Get the torque exerted on the end effector in Newtons*meters (relative to the end effector).

        Returns:
            list: [Tx,Ty,Tz]
        """
        return self.tool_wrench.get()["torque"]
        
    def move_global(self, cords, speed=.05):
        """
        Move to cords given at the speed given (m/s)
        """
        curr_pos = self.read_global_pos()
        vel_unscaled = curr_pos[0:3]
        for i in range(3):
            vel_unscaled[i] = float(cords[i] - curr_pos[i])
        vel = self.normalize(vel_unscaled, speed)
        #print(vel)
        vel_joints = self.solve_ik_velo(vel)
        for i in range(len(vel_joints)):
            if vel_joints[i]>1.5:
                raise ValueError("could not solve IK with small angles: changes - " + str(vel_joints))
        
        distance = 0
        for i in range(len(vel_unscaled)):
            distance = distance + vel_unscaled[i]**2
        distance = math.sqrt(distance)

        self.set_vel(vel_joints.tolist())
        time.sleep(distance/speed)
        self.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Direction scaled to distance inverse solve at location it reaches before next velo command
        # distance = speed * self.frequency
        # scaled_dir = self.normalize(direction, distance)                        # FLAG THIS LINE TO LOOK MORE INTO BEST DISTANCE AHEAD
        
        # curr_pos = self.read_global_pos()
        # next_pos = curr_pos
        # for i in range(3):
        #     next_pos[i] = float(curr_pos[i] + scaled_dir[i])

        # next_angles = self.solve_ik(next_pos) # THIS LINE WHAT TYPE FOR NEXT POS
        # curr_angles = self.read_joints_pos()
        # if next_angles is None:
        #     raise TypeError("next pos ik_solve returned type none")
        # #print("next angles: " + str(next_angles))
        # angle_changes = [0.0,0.0,0.0,0.0,0.0,0.0]
        # for i in range(len(next_angles)):
        #     angle_changes[i] = next_angles[i]-curr_angles[i]
        #     if angle_changes[i] > 2: #in radians
        #         print("could not solve IK with small angles")
        #         print("angle changes: " + str(angle_changes))
        #         self.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #         raise ValueError("could not solve IK with small angles: changes - " + str(angle_changes))
        # velo = self.normalize(angle_changes, speed)
        # self.set_vel(velo)
    
    def start_xyz_rel(self, cord_change, speed=.05):
        if speed > 2:
            self.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            raise ValueError("Speed too high given in start_xyz_rel: " + str(speed))
        curr_pos = self.read_global_pos()
        cords = [0,0,0]
        for i in range(3):
            cords[i] = curr_pos[i] + cord_change[i]
        print("cords: " + str(cords))
        vel_unscaled = curr_pos[0:3]
        print("vel_unscaled_primary: " + str(vel_unscaled))
        for i in range(3):
            vel_unscaled[i] = float(cords[i] - curr_pos[i])
            print("vel_unscaled " + str(i) + " "+ str(vel_unscaled))
        dist = vel_unscaled.copy()
        print("dist: " + str(dist))
        vel = self.normalize(vel_unscaled, speed)
        print("dist: " + str(dist))
        #print(vel)
        vel_joints = self.solve_ik_velo(vel)
        for i in range(len(vel_joints)):
            if vel_joints[i]>1.5:
                self.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                raise ValueError("could not solve IK with small angles: changes - " + str(vel_joints))
        distance = 0
        for i in range(len(vel_unscaled)):
            distance = distance + dist[i]**2
        distance = math.sqrt(distance)

        self.set_vel(vel_joints.tolist())
        return distance/speed
        
    def move_to_angles(self, point, max_speed = .05, degrees = False):
        """
        moves to point by calculating angle changes and time.
        paramter max_speed in radians/sec
        """
        curr_joints = self.read_joints_pos()
        print("curr_joints: " +str(curr_joints))
        change_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        largest_angle = 0
        
        for i in range(len(curr_joints)):
            if degrees:
                point[i] = float((point[i]/180)*3.141592653);
            change_joints[i] = point[i] - curr_joints[i]
            if change_joints[i] > 6: # If angle change is greater than ~300 degrees
                raise ValueError("requires move over 2pi radians. If in degrees set degrees = True")
            if change_joints[i] > largest_angle:
                largest_angle = change_joints[i]
        print("change_joints: " + str(change_joints))
        joint_speeds = [x *(max_speed/largest_angle) for x in change_joints]
        print("joint_speeds: " +str(joint_speeds))
        time_to_move = largest_angle/max(joint_speeds)
        print("time to move: " + str(time_to_move))
        print("largest angle: " + str(largest_angle))
        print("joint_speeds: " + str(joint_speeds))
        self.set_vel(joint_speeds)
        time.sleep(time_to_move)
        self.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(.2)
        print("moved to position " + str(point))
        # if any(i > .001 for i in [xi - yi for xi, yi in zip(self.read_joints_pos(),point)]):
        #     self.move_to_angles(point, max_speed=(max_speed/5), degrees=degrees)
                
    def experiment(self, num):
        if num>3:
            raise ValueError("Num too high")
            print("2")
        print("3")
        
    
    def normalize(self, vector, length):
        mag = 0
        for i in range(len(vector)):
            mag = mag + vector[i]**2
        mag = math.sqrt(mag)
        vector_new = vector
        for i in range(len(vector)):
            vector_new[i] = (vector[i]/mag)*length
        return vector_new


class JointStates(rclpy.node.Node):
    """
    Subscribe to the joint_states topic.
    """

    def __init__(self):
        """
        Initialize the JointStates node.
        """
        super().__init__("joint_state_subscriber")
        self.subscription = self.create_subscription(
            JointState, "joint_states", self.listener_callback, 10
        )
        self.ik_solver = URKinematics("ur3e")
        self.states = None
        self.done = False

    def listener_callback(self, msg):
        """
        Callback for when joint states are received.

        Args:
            msg (JointState): The joint state message.
        """
        data = {
            "name": [msg.name[5]] + msg.name[0:5],
            "position": [msg.position[5]] + msg.position[0:5].tolist(),
            "velocity": [msg.velocity[5]] + msg.velocity[0:5].tolist(),
            "effort": [msg.effort[5]] + msg.effort[0:5].tolist(),
        }

        self.states = data
        self.done = True

    def get_joints(self):
        """
        Get the current joint states.

        Returns:
            dict: The current joint states.
        """
        self.wait(self)
        self.done = False
        return self.states
        
    def wait(self, client):
        """
        Wait for the joint states to be updated.

        Args:
            client (Node): The node to wait for.
        """
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for joint_states_client")



class ToolWrench(rclpy.node.Node):
    """
    Subscribe to the /force_torque_sensor_broadcaster/wrench topic.
    Data =  {
            header = [sec, nanosec],
            force = [x,y,z],
            torque = [x,y,z]
            }
    """

    def __init__(self):
        """
        Initialize the ToolWrench node.
        """
        super().__init__("wrench_subscriber")
        self.subscription = self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.listener_callback,
            10,
        )
        self.states = None
        self.done = False

    def listener_callback(self, msg):
        """
        Callback for when wrench data is received.

        Args:
            msg (WrenchStamped): The wrench stamped message.
        """
        data = {
            "header": [msg.header.stamp.sec, msg.header.stamp.nanosec],
            "force": [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
            "torque": [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z],
        }

        self.states = data
        self.done = True

    def get(self):
        """
        Get the current wrench data.

        Returns:
            dict: The current wrench data.
        """
        self.wait(self)
        self.done = False
        return self.states

    def wait(self, client):
        """
        Wait for the wrench data to be updated.

        Args:
            client (Node): The node to wait for.
        """
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for wrench client")
