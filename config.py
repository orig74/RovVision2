#Joystick configuration
#mode 2
class Joy_map:
    ud=1
    yaw=0
    fb=4
    lr=3
    shift_bt=4
    shift2_bt=5
    record_bt=10
    arm_disarm=9
    depth_hold_bt=1

#cameras info
cam_resx,cam_resy=1920,1200
cam_res_rgbx,cam_res_rgby=cam_resx//2,cam_resy//2
gst_bitrate=1024*2
fps=10

#gstreamer 
gst_ports=[6760,6761]


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
