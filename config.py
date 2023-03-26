#cameras info
import os
rov_type = int(os.environ.get('ROV_TYPE',4))
is_sim = 'SIM' in os.environ
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
    cam_main_sx,cam_main_sy=640,480

cam_fps=8
send_modulo=1

#gstreamer
gst_ports=[6760,6761]
gst_bitrate=1024*3
#gst_bitrate=256
gst_speed_preset=1

#tracker type
#tracker = 'local'
tracker = 'rope'

joy_deadband=0.00
joy_dtarget_min=0.05

manual_control_limit=0.85
thruster_limit=1.0

viewer_blacks=(50,100)

save_modulo=5

if __name__=='__main__':
    #for bash scripts to get state
    import sys
    print(globals()[sys.argv[1]],end='')
