### PIDS
depth_pid={\
        'P':2.5,
        'I':0.001,
        'D':5,
        'limit':0.3,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':False,
        'initial_i':0,
        'func_in_err':None}

yaw_pid={\
        'P':2.5,
        'I':0.001,
        'D':5,
        'limit':0.6,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':True,
        'initial_i':0,
        'func_in_err':None}

pitch_pid=roll_pid=yaw_pid

#if set to true always try to mantain 0 roll
roll_target_0 = True

pos_pids=[depth_pid]*3
