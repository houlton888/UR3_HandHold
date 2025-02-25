1) Turn on uR3e arm
2) Ensure ubuntu computer is on, go to 130.64.16.222 sign in and pull this git
3) in one terminal run launch.py
4) In another terminal run lead_arm_bounded.py for a program where the arm follows you. Length can be set as run_time variable in seconds

Notes:
- To launch gripper and open then close it (so it can grab something) run the following in new terminal:
    $ cd myur
    $ python3 Gripper_Node.py --ip "130.64.17.95"
- human_response.py is what I used to collect data for my study
- velo_control creates the class for the robot arm and has the following useful functions (more about them can be found below function def in file):
r.set_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
r.get_force()
r.read_global_pos()
r.read_joints_pos()
r.solve_ik(global_pos)  THIS GIVES np.floats
r.solve_fk(angles)
r.move_to_angles([30, -180, 90, -90, 60, 0], max_speed = .05, degrees = True)
r.move_global(xyz_pos) #rx ry rz stays the same? #DOESNT WORK WELL
t_sleep = r.start_xyz_rel([x,y,z]) (in_out,left_right,up_down)
