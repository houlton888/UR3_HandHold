from velo_control import Arm_Velocity   #(row,col)   Z = up   Y = left
import numpy as np
import time
import pandas as pd
import datetime
from openpyxl import load_workbook

trial_number = 3
Name = "person2"
date = "11-24-2024"
dominant_hand = "R"

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

time.sleep(.25)
#-----------------------------------------------------------------------
movement_number = 2 # circle clockwise

circle_time = 6 # time to make circle in sec
freq = frequency*5
dp = 0
info = pd.DataFrame({'Movement'+str(movement_number):['Circle counter clock, Radius: '+str(radius)]})
data = pd.concat([data, info], axis=1) 

frc = np.zeros((int(circle_time/freq), 3)) # [row][column]
pos = np.zeros((int(circle_time/freq), 3)) # [row][column]
time_arr = np.zeros(int(circle_time/freq))

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
info = pd.DataFrame({'Movement'+str(movement_number):['Isosolese triangle to left, base and height = Radius: '+str(radius)]})
data = pd.concat([data, info], axis=1) 

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

movement = pd.DataFrame({'Time'+str(movement_number): time_arr, 
        'posx'+str(movement_number):pos[:,0], 'posy'+str(movement_number):pos[:,1], 'posz'+str(movement_number):pos[:,2],
        'fx'  +str(movement_number):frc[:,0], 'fy'  +str(movement_number):frc[:,1], 'fz'  +str(movement_number):frc[:,2]
})
data = pd.concat([data, movement], axis=1) 

r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .1, degrees = True)
time.sleep(.25)
#------------------------------------------------------------------------------------------
movement_number = 4 # constant accel from 0

radius = .13
y_0 = r.read_global_pos()[1]
t_sleep = r.start_xyz_rel([0,radius,0])
time.sleep(t_sleep)
dp = 0
vel = 0
accel = .05
freq = frequency*2
info = pd.DataFrame({'Movement'+str(movement_number):['Constant accel then decel left to right '+str(radius)]})
data = pd.concat([data, info], axis=1)

frc = np.zeros((218, 3)) # [row][column]
pos = np.zeros((218, 3)) # [row][column]
time_arr = np.zeros(218)

while r.read_global_pos()[1] > y_0:
    frc[dp] = np.subtract(r.get_force(), start_force)
    pos[dp] = r.read_global_pos()[0:3]
    time_arr[dp] = time.time()-t_start
    vel = vel + accel*freq
    joint_velos = r.solve_ik_velo([0,-vel,0]).tolist()
    r.set_vel(joint_velos)
    time.sleep(freq)
    dp = dp + 1
dp2 = 0
for i in range(dp):
    frc[dp+dp2] = np.subtract(r.get_force(), start_force)
    pos[dp+dp2] = r.read_global_pos()[0:3]
    time_arr[dp+dp2] = time.time()-t_start
    vel = vel - accel*freq
    joint_velos = r.solve_ik_velo([0,-vel,0]).tolist()
    r.set_vel(joint_velos)
    time.sleep(freq)
    dp2 = dp2 + 1
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

movement = pd.DataFrame({'Time'+str(movement_number): time_arr, 
        'posx'+str(movement_number):pos[:,0], 'posy'+str(movement_number):pos[:,1], 'posz'+str(movement_number):pos[:,2],
        'fx'  +str(movement_number):frc[:,0], 'fy'  +str(movement_number):frc[:,1], 'fz'  +str(movement_number):frc[:,2]
})
data = pd.concat([data, movement], axis=1) 
time.sleep(.1)

#-------------------------------------------------------------------------------------------------------
movement_number = 5 # away and back to middle constant velo
info = pd.DataFrame({'Movement'+str(movement_number):['Away and back to middle constant velo (.05m/s)']})
data = pd.concat([data, info], axis=1) 

t_sleep = r.start_xyz_rel([.1,radius,0], speed=.05) # This starts a movements velocity for a relative movement, need to make zero when time to arrive at xyz global is done
dp = 0
t_start1 = time.time()
freq=frequency*2
frc = np.zeros((int(t_sleep/freq + 1), 3)) # [row][column]
pos = np.zeros((int(t_sleep/freq + 1), 3)) # [row][column]
time_arr = np.zeros(int(t_sleep/freq + 1))
while time.time()-t_start1 < t_sleep: 
    # record total time of script, pos as tuple, and force as tuple into array
    frc[dp] = np.subtract(r.get_force(), start_force)
    pos[dp] = r.read_global_pos()[0:3]
    time_arr[dp] = time.time()-t_start
    dp = dp+1
    time.sleep(freq)
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

movement = pd.DataFrame({'Time'+str(movement_number): time_arr, 
            'posx'+str(movement_number):pos[:,0], 'posy'+str(movement_number):pos[:,1], 'posz'+str(movement_number):pos[:,2],
            'fx'  +str(movement_number):frc[:,0], 'fy'  +str(movement_number):frc[:,1], 'fz'  +str(movement_number):frc[:,2]
    })
data = pd.concat([data, movement], axis=1)

time.sleep(.1)

#----------------------------------------------------------
movement_number = 6 # Constant accel in to person

radius = .1
x_0 = r.read_global_pos()[0] - radius
dp = 0
vel = 0
accel = .05
freq = frequency*2
info = pd.DataFrame({'Movement'+str(movement_number):['Constant accel towards person, length: '+str(radius)]})
data = pd.concat([data, info], axis=1)

frc = np.zeros((200, 3)) # [row][column]
pos = np.zeros((200, 3)) # [row][column]
time_arr = np.zeros(200)

while r.read_global_pos()[0] > x_0:
    frc[dp] = np.subtract(r.get_force(), start_force)
    pos[dp] = r.read_global_pos()[0:3]
    time_arr[dp] = time.time()-t_start
    vel = vel + accel*freq
    joint_velos = r.solve_ik_velo([-vel,0,0]).tolist()
    r.set_vel(joint_velos)
    time.sleep(freq)
    dp = dp + 1
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

movement = pd.DataFrame({'Time'+str(movement_number): time_arr, 
            'posx'+str(movement_number):pos[:,0], 'posy'+str(movement_number):pos[:,1], 'posz'+str(movement_number):pos[:,2],
            'fx'  +str(movement_number):frc[:,0], 'fy'  +str(movement_number):frc[:,1], 'fz'  +str(movement_number):frc[:,2]
    })
data = pd.concat([data, movement], axis=1)
r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .1, degrees = True)
time.sleep(.1)

#-----------------------------------------------------------
movement_number = 7 # sin wave left to right
radius = .12
info = pd.DataFrame({'Movement'+str(movement_number):['Sin left to right length: '+str(radius)]})
data = pd.concat([data, info], axis=1)

t_sleep = r.start_xyz_rel([0,radius,0])
time.sleep(t_sleep)
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
freq = frequency*5
print(freq)
dp = 0
step = .01*freq
points = radius*2/step

frc = np.zeros((points, 3)) # [row][column]
pos = np.zeros((points, 3)) # [row][column]
time_arr = np.zeros(points)

for i in range(int(radius*2/step)):
    frc[dp] = np.subtract(r.get_force(), start_force)
    pos[dp] = r.read_global_pos()[0:3]
    time_arr[dp] = time.time()-t_start
    t_step = r.start_xyz_rel([0,-step,(step*2)*np.cos(.05*i)],speed=.1)
    dp = dp+1
    time.sleep(t_step)

movement = pd.DataFrame({'Time'+str(movement_number): time_arr, 
        'posx'+str(movement_number):pos[:,0], 'posy'+str(movement_number):pos[:,1], 'posz'+str(movement_number):pos[:,2],
        'fx'  +str(movement_number):frc[:,0], 'fy'  +str(movement_number):frc[:,1], 'fz'  +str(movement_number):frc[:,2]
})
data = pd.concat([data, movement], axis=1) 
time.sleep(.1)
#-----------------------------------------------------------
df = pd.DataFrame.from_dict(data)
# for first time to reset sheet only:
df.to_excel('output.xlsx', index=False, sheet_name=('trial'+str(trial_number)+str(Name)))

#Every subsequent test:
#with pd.ExcelWriter('output.xlsx', engine='openpyxl', mode='a', if_sheet_exists='new') as writer:
#    df.to_excel(writer, sheet_name=('trial'+str(trial_number)+str(Name)), index=False)

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
r.move_global(xyz_pos) #rx ry rz stays the same? #DOESNT WORK WELL
t_sleep = r.start_xyz_rel([x,y,z]) (in_out,left_right,up_down)
"""