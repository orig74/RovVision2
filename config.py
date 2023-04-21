#cameras info
import os
rov_type = int(os.environ.get('ROV_TYPE',4))
is_sim = 'SIM' in os.environ
is_sim_zmq = os.environ.get('SIM_STREAM_TYPE','')=='ZMQ'
sim_type = os.environ.get('SIM','')
camera_setup='stereo' #'mono'

if rov_type==1:
    cam_resx,cam_resy=1920,1200
    #cam_resx,cam_resy=1280,1024
    cam_res_rgbx,cam_res_rgby=cam_resx//2,cam_resy//2
    reverse_camera_order=True
if rov_type==2:
    cam_resx,cam_resy=1280,1024
    cam_res_rgbx,cam_res_rgby=cam_resx//2,cam_resy//2
    reverse_camera_order=False
if rov_type==3:
    cam_resx,cam_resy=1920,1200#1080 todo change to 1080 for bluerobotics cameras
    cam_res_rgbx,cam_res_rgby=cam_resx//2,cam_resy//2
    camera_setup='mono' #'mono'
if rov_type==4:
    #cam_resx,cam_resy=1280,1024
    #original cam res in beyer format 2464x2056
    cam_resx,cam_resy=2464//2,2056//2
    cam_res_rgbx,cam_res_rgby=cam_resx//2,cam_resy//2
    cam_res_gst_pad_lines=2 ## added incase of hight not diveded by 4
    cam_main_sx,cam_main_sy=848,480
    cam_main_dgui_sx,cam_main_dgui_sy=848//2,240
    cam_main_gui_sx,cam_main_gui_sy=640,int(640/848.*480)
    water_scale=90/64.2
    #water_scale=1
    cam_main_int = [
        [424*water_scale,0,848/2],
        [0,422*water_scale,480/2],
        [0,0,1]]
    grip_pos_rel_mm=(220,-17,21.5) #range left up in mm
    #grip_pos_rel_mm=(1220,20,20) #range left up in mm
    valid_range_mm=(50,1500)
    
cam_fps=8
send_modulo=1

#gstreamer
gst_ports=[6760,6761]
gst_cam_main_port = 17894
gst_cam_main_depth_port = 17895
gst_bitrate=1024*3
#gst_bitrate=256
gst_speed_preset=1

#tracker type
#tracker = 'local'
tracker = 'rope'

joy_deadband=0.00
joy_dtarget_min=0.05

manual_control_limit=0.4 #limit in manual plugin override by controller limit
thruster_limit=1.0 #limit in esp32 gate
thruster_limit_controler=1.0 #limit in controller

viewer_blacks=(50,100)

save_modulo=5

if __name__=='__main__':
    #for bash scripts to get state
    import sys
    print(globals()[sys.argv[1]],end='')
