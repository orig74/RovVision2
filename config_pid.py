### PIDS
ds=0.1
depth_pid={\
        'P':2.5*ds,
        'I':0.001*ds,
        'D':5*ds,
        'limit':0.2,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':False,
        'initial_i':0,
        'func_in_err':None}

ys=0.1
yaw_pid={\
        'P':2.5*ys,
        'I':0.001*ys,
        'D':5*ys,
        'limit':0.2,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':True,
        'initial_i':0,
        'func_in_err':None}

rs=0.0
roll_pid={\
        'P':2.5*rs,
        'I':0.001*rs,
        'D':5*rs,
        'limit':0.2,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':True,
        'initial_i':0,
        'func_in_err':None}

ps=0.0
pitch_pid={\
        'P':2.5*ps,
        'I':0.001*ps,
        'D':5*ps,
        'limit':0.2,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':True,
        'initial_i':0,
        'func_in_err':None}

#if set to true always try to mantain 0 roll
roll_target_0 = True

sc=4
pos_pid={\
        'P':2.5 * sc ,
        'I':0.001 * sc ,
        'D':5 * sc ,
        'limit':0.6,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':True,
        'initial_i':0,
        'func_in_err':None}


pos_pids=[pos_pid]*3
