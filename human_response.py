from velo_control import Arm_Velocity   #(row,col)   Z = up   Y = left
import numpy as np
import time
import pandas as pd
import datetime

trial_number = 1
Name = "person1"
date = "11-24-2024"
dominant_hand = "R"

time_arr = [0, .1, .2]

frequency = .01
r = Arm_Velocity(pub_freq = frequency)
r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .1, degrees = True)
time.sleep(.25)
start_force = r.get_force()
print("Ready to start")
while start_force[2] - r.get_force()[2] < 2:
    time.sleep(.02)
t_start = time.time()
now = datetime.datetime.now()
current_time = now.time()
formatted_time = current_time.strftime("%H:%M.%S")

data = pd.DataFrame({'Person': [Name, date, 'time: ' + str(formatted_time), str(dominant_hand)+' Hand']})
#data = pd.DataFrame({'Person': [Name, date, dominant_hand], 'Date': [date], 'Hand Used (R/L)': [dominant_hand]})
print("Starting")
# for each step movement
#------------------------------------------------------------------------
movement_number = 1 # constant velo 1 direction
info = pd.DataFrame({'Movement'+str(movement_number):['Right constant velo']})
data = pd.concat([data, info], axis=1) 
radius = .08
t_sleep = r.start_xyz_rel([0,-radius,0]) # This starts a movements velocity for a relative movement, need to make zero when time to arrive at xyz global is done
print("sleep: " + str(t_sleep))
dp = 0
frc = np.zeros((int(t_sleep/frequency + 1), 3)) # [row][column]
pos = np.zeros((int(t_sleep/frequency + 1), 3)) # [row][column]
time_arr = np.zeros(int(t_sleep/frequency + 1))
while time.time()-t_start < t_sleep: 
    # record total time of script, pos as tuple, and force as tuple into array
    frc[dp] = np.subtract(r.get_force(), start_force)
    pos[dp] = r.read_global_pos()[0:3]
    time_arr[dp] = time.time()-t_start
    dp = dp+1
    time.sleep(frequency)
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

movement = pd.DataFrame({'Time'+str(movement_number): time_arr,  
        'posx'+str(movement_number):pos[:,0], 'posy'+str(movement_number):pos[:,1], 'posz'+str(movement_number):pos[:,2],
        'fx'  +str(movement_number):frc[:,0], 'fy'  +str(movement_number):frc[:,1], 'fz'  +str(movement_number):frc[:,2]
})
data = pd.concat([data, movement], axis=1) 
#set velo to next thing
#time.sleep(.01)
time.sleep(.25)
#-----------------------------------------------------------------------
movement_number = 2 # circle clockwise
info = pd.DataFrame({'Movement'+str(movement_number):['Circle counter clock, Radius: '+str(radius)]})
data = pd.concat([data, info], axis=1) 
circle_time = 6 # time to make circle in sec
freq = frequency*5
dp = 0
frc = np.zeros((int(circle_time/freq + 1), 3)) # [row][column]
pos = np.zeros((int(circle_time/freq + 1), 3)) # [row][column]
time_arr = np.zeros(int(circle_time/freq + 1))
for i in range(int(circle_time/freq)):
    step = i*2*np.pi*freq/(circle_time)
    t_step = r.start_xyz_rel([0,(freq*radius)*np.sin(step),(freq*radius)*np.cos(step)],speed=(2*np.pi*radius/circle_time))
    frc[dp] = np.subtract(r.get_force(), start_force)
    pos[dp] = r.read_global_pos()[0:3]
    time_arr[dp] = time.time()-t_start
    dp = dp+1
    time.sleep(t_step)

t_sleep = r.start_xyz_rel([0,radius,0])
time.sleep(t_sleep)
r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .1, degrees = True)
movement = pd.DataFrame({'Time'+str(movement_number): time_arr, 
        'posx'+str(movement_number):pos[:,0], 'posy'+str(movement_number):pos[:,1], 'posz'+str(movement_number):pos[:,2],
        'fx'  +str(movement_number):frc[:,0], 'fy'  +str(movement_number):frc[:,1], 'fz'  +str(movement_number):frc[:,2]
})
data = pd.concat([data, movement], axis=1) 

time.sleep(.25)
#----------------------------------------------------------------------------------------
movement_number = 3 # Triangle start left

freq = frequency*5
dp = 0
frc = np.zeros((int(circle_time/freq + 1), 3)) # [row][column]
pos = np.zeros((int(circle_time/freq + 1), 3)) # [row][column]
time_arr = np.zeros(int(circle_time/freq + 1))
for i in [[0, radius, 0],[0, -radius/2, radius], [0, -radius/2, -radius]]:
    t_sleep = r.start_xyz_rel(i)
    t_begin = time.time()
    while time.time()-t_begin < t_sleep: 
        frc[dp] = np.subtract(r.get_force(), start_force)
        pos[dp] = r.read_global_pos()[0:3]
        time_arr[dp] = time.time()-t_start
        dp = dp+1
        time.sleep(freq)

r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .1, degrees = True)
time.sleep(.25)
#------------------------------------------------------------------------------------------
movement_number = 4 # constant accel from 0
y_0 = r.read_global_pos()[1]
t_sleep = r.start_xyz_rel([0,radius,0])
time.sleep(t_sleep)
dp = 0
vel = 0
accel = .1
freq = frequency*5
# while r.read_global_pos()[1] > y_0:
#     vel = vel + .001
#     joint_velos = r.solve_ik_velo([0,-vel,0]).tolist()
#     r.set_vel(joint_velos)
#     time.sleep(freq)
#     dp = dp + 1

# for i in range(dp):
#     vel = vel - accel*freq
#     joint_velos = r.solve_ik_velo([0,-vel,0]).tolist()
#     r.set_vel(joint_velos)
#     time.sleep(freq)
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


movement_number = 5 # triangle
movement_number = 6 # sin motion

df = pd.DataFrame.from_dict(data)
df.to_excel('output.xlsx', index=False, sheet_name=('trial'+str(trial_number)+str(Name)))


r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
r.destroy_node()

""" Functions:
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
r.get_force()
r.read_global_pos()
r.read_joints_pos()
r.solve_ik(global_pos)  THIS GIVES np.floats
r.solve_fk(angles)
r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .05, degrees = True)
r.move_global(xyz_pos) #rx ry rz stays the same?
t_sleep = r.start_xyz_rel([x,y,z]) (in_out,left_right,up_down)
"""