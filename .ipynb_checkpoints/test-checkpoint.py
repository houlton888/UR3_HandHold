from velo_control import Arm_Velocity
import numpy as np
import time
import math

r = Arm_Velocity(pub_freq = .05)
r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .1, degrees = True)
time.sleep(.25)

print(r.read_global_pos()[1])
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
print(r.read_global_pos())
r.destroy_node()

