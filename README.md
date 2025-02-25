1) Turn on uR3e arm
2) Ensure ubuntu computer is on, go to 130.64.16.222 sign in and pull this git
3) in one terminal run launch.py
4) In another terminal run lead_arm_bounded.py for a program where the arm follows you. Length can be set as run_time variable in seconds

Notes:
- To launch gripper and open then close it (so it can grab something) run the following in new terminal:
    cd myur
    python3 Gripper_Node.py --ip "130.64.17.95"
- human_response.py is what I used to collect data for my study
