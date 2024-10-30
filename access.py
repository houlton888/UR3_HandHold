from velo_control import Arm_Velocity
import numpy as np
import time
import math
import matplotlib.pyplot as plt
"""
To-Dos - apply filter so deriv_error uses moving average of past 5 derivs or something. Run calcs at 5x send rate?
"""
run_time = 10 # in seconds
frequency = .1 # Make sure force measure, and send command both happen at the beginning of the frequency
max_speed_arm = .2 # This is in r/sec
zero_force = np.array([0.0, 0.0, 0.0]) # Z-value is force in newtons that the hands press against eachother
Kp = .1;
Ki = 0;
Kd = 0;

r = Arm_Velocity(pub_freq = frequency)
error_arr = np.zeros((int(run_time/frequency + 1), 3)) # [row][column]
t_start = time.time()
r.move_to_angles([30, -180, 90, -90, 60, 0], degrees = True)
start_force = r.get_force()

print("starting feedback")
loop_num = 0;
int_error = 0;
direction = [0.0, 0.0, 0.0]
vel = np.zeros(3)

while time.time()-t_start < run_time:
    loop_num = loop_num + 1
    time.sleep(frequency)
    curr_force = np.array([r.get_force()])
    error_arr[loop_num] = np.subtract(curr_force, np.add(zero_force, start_force))   # All are np arrays so can just add subtract normally?
    error = error_arr[loop_num]
    print("Force error: " + str(error))
    int_error = np.add(int_error, error)
    deriv_error = np.divide(np.subtract(error, error_arr[loop_num-1]),frequency)

    accel = error*Kp + int_error*Ki + deriv_error*Kd
    vel = vel + accel*frequency
    speed = math.sqrt(sum(pow(num, 2) for num in vel))
    y =  vel[0] #direction ratio of x direction--|
    z = -vel[1] #direction ratio of y direction  |-> Converts cord-system of the force sensor to the global cord-system of the arm
    x = -vel[2] #direction ratio of z direction--|
    
    # If speed is too high limit to max speed
    if speed > max_speed_arm:
        print("speed to high, limiting to max")
        speed = max_speed_arm
    
    # If speed is small ignore (adds deadband) (accel <.5 newtons)
    if math.sqrt(sum(pow(num, 2) for num in accel)) > .01:
        r.move([x,y,z], speed)

r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
r.destroy_node()

#error_arr.tofile('data2.csv', sep = ',')
""" Functions:
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
r.read_global_pos()
r.read_joints_pos()
r.solve_ik(global_pos)  THIS GIVES np.floats
r.solve_fk(angles)
r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .05, degrees = True)
"""
