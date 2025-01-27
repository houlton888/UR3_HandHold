from velo_control import Arm_Velocity
import numpy as np
import time
import math

"""
To-Dos - apply filter so deriv_error uses moving average of past 5 derivs or something. Run calcs at 5x send rate?
"""
run_time = 10 # in seconds
frequency = .005 # Make sure force measure, and send command both happen at the beginning of the frequency
max_speed_arm = .2 # This is in r/sec
hand_pressure = 6 # in Newtons
return_accel = .75 # m/s^2
zero_force = np.array([0.0, 0.0, -hand_pressure]) # Z-value is force in newtons that the hands press against eachother
Kp = .15; #.2 triggers autostop
Ki = 0*0.001;
Kd = 0*0.00002;
x_max_distance = .075
y_max_distance = .15
z_max_distance = .1

r = Arm_Velocity(pub_freq = frequency)
error_arr = np.zeros((int(run_time/frequency + 1), 3)) # [row][column]
r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .1, degrees = True)
time.sleep(.25)
start_pos = np.array(r.read_global_pos()[0:3])
start_force = r.get_force()

print("Ready to start")
while start_force[2] - r.get_force()[2] < (hand_pressure - .2):
    time.sleep(.02)

t_start = time.time()
loop_num = 0;
int_error = 0;
vel = np.zeros(3)

while time.time()-t_start < run_time:
    loop_num = loop_num + 1
    time.sleep(frequency)
    curr_force = np.array(r.get_force())
    error = np.subtract(curr_force, np.add(zero_force, start_force)) # All are np arrays so can just add subtract normally?
    error_arr[loop_num] = error
    print("Force error: " + str(error))
    int_error = np.add(int_error, error)
    deriv_error = np.divide(np.subtract(error, error_arr[loop_num-1]),frequency)

    accel = error*Kp + int_error*Ki + deriv_error*Kd #np xyz array
    
    # Calculate position:
    position = np.array(r.read_global_pos()[0:3]) - start_pos
    print("xyz position:  " + str(position))

    # Check if the new position is within bounds
    if position[0] > x_max_distance:
        accel[2] = accel[2]+return_accel
    if position[0] < -x_max_distance:
        accel[2] = accel[2]-return_accel
        
    if position[1] > y_max_distance:
        print("OOOOOVVVEEERRRRR")
        accel[0] = accel[0]-return_accel
    if position[1] < -y_max_distance:
        accel[0] = accel[0]+return_accel

    if position[2] > z_max_distance:
        accel[1] = accel[1]+return_accel
    if position[2] < -z_max_distance:
        accel[1] = accel[1]-return_accel


    # #get calculated position error from actual
    # pos = np.subtract(r.read_global_pos(), start_pos)
    # print("position error:  " + str((np.subtract(pos[0:3], positions))))
    # pos_err = np.append(pos_err, (np.subtract(pos[0:3], positions)), axis=0)
    
    vel = vel + accel*frequency
    print("vel: " +str(vel))
    speed = math.sqrt(sum(pow(num, 2) for num in vel))
    # If speed is too high limit to max speed
    if speed > max_speed_arm:
        print("speed to high, limiting to max")
        vel = r.normalize(vel, max_speed_arm) # scales magnitude of vel vector to be length max_speed
        
    y =  vel[0] #direction ratio of x direction-|
    z = -vel[1] #direction ratio of y direction |-> Converts cord-system of the force sensor to the global cord-system of the arm
    x = -vel[2] #direction ratio of z direction-|

    # THIS DOENST WORK?
    # #account for if vel really small set to 0
    # if np.all(vel<.001):
    #     vel = np.zeros(3)
        
    # If speed is small ignore (adds deadband) (accel <.5 newtons)
    if math.sqrt(sum(pow(num, 2) for num in accel)) > .2:
        joint_velos = r.solve_ik_velo([x,y,z]).tolist()
        print("sent velos:  " + str(joint_velos))
        r.set_vel(joint_velos)
        
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
r.destroy_node()