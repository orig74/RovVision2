### PIDS
ds=0.1
depth_pid={
        'P':2.5*ds,
        'I':0.002*ds,
        'D':0*ds,
        'limit':0.2,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':False,
        'initial_i':0,
        'func_in_err':None}

ss=0.1
sonar_pid={
        'P':2.5*ss,
        'I':0.05*ss,
        'D':0*ss,
        'limit':0.2,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':False,
        'initial_i':0,
        'func_in_err':None}

ys=0.04
yaw_pid={
        'P':2*ys,
        'I':0.04*ys,
        'D':0.4*ys,
        'limit':0.5,
        'step_limit':0.05,
        'i_limit':0.1,
        'FF':0,
        'angle_deg_type':True,
        'initial_i':0,
        'func_in_err':None}

rs=0.01
roll_pid={
        'P':1*rs,
        'I':0.02*rs,
        'D':0.2*rs,
        'limit':0.2,
        'step_limit':0.05,
        'i_limit':0.1,
        'FF':0,
        'angle_deg_type':True,
        'initial_i':0,
        'func_in_err':None}

ps=0.006
pitch_pid={
        'P':1*ps,
        'I':0.1*ps,
        'D':1*ps,
        'limit':0.7,
        'step_limit':0.05,
        'i_limit':0.7,
        'FF':0,
        'angle_deg_type':True,
        'initial_i':0,
        'func_in_err':None}

#if set to true always try to mantain 0 roll
roll_target_0 = True

sc=0.03
pos_pid_x={
        'P':2.5 * sc ,
        'I':0.001 * sc ,
        'D':5 * sc ,
        'limit':0.6,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':False,
        'initial_i':0,
        'func_in_err':None}
sc=0.25
pos_pid_y={
        'P':2.5 * sc ,
        'I':0.001 * sc ,
        'D':5 * sc ,
        'limit':0.6,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':False,
        'initial_i':0,
        'func_in_err':None}

sc=0.00
pos_pid_z={
        'P':2.5 * sc ,
        'I':0.001 * sc ,
        'D':5 * sc ,
        'limit':0.6,
        'step_limit':0.05,
        'i_limit':0.01,
        'FF':0,
        'angle_deg_type':False,
        'initial_i':0,
        'func_in_err':None}





pos_pids=[pos_pid_x, pos_pid_y, pos_pid_z]
