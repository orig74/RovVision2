#Joystick configuration
#mode 2
class Joy_map:
    ud=1
    yaw=0
    fb=4
    lr=3
    shift1_bt=4
    shift2_bt=5
    record_bt=10
    arm_disarm=9
    depth_hold_bt=1
    att_hold_bt=2

#cameras info
cam_resx,cam_resy=1920,1200
cam_res_rgbx,cam_res_rgby=cam_resx//2,cam_resy//2
gst_bitrate=1024*2
fps=10

#gstreamer 
gst_ports=[6760,6761]

