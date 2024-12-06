from velo_control import Arm_Velocity   # X = in   Y = left   Z = up
import numpy as np
import time

frequency = .01
r = Arm_Velocity(pub_freq = frequency)

#Code here to control arm




r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
r.destroy_node()

""" Functions:
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #tells each joint what vel to go to
r.read_global_pos()
r.read_joints_pos()
joint_velos = r.solve_ik_velo([x_vel,y_vel,z_vel]).tolist() #returns joint velocities as function of xyz velocity
r.set_vel(joint_velos)
r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .05, degrees = True) # starting position
r.move_global(xyz_pos) #rx ry rz stays the same?
t_sleep = r.start_xyz_rel([x_distance,y_distance,z_distance])  #moves relative distances given after time t_sleep 
SET VELOCITY TO 0 AFTER TIME T_SLEEP HAS PASSED!
"""