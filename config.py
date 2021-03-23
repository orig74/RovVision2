#cameras info
import os
rov_type = int(os.environ.get('ROV_TYPE',1))
camera_setup='stereo' #'mono'

if rov_type==1:
    cam_resx,cam_resy=1920,1200
    reverse_camera_order=True
if rov_type==2:
    cam_resx,cam_resy=1280,1024
    reverse_camera_order=False
if rov_type==3:
    cam_resx,cam_resy=1920,1200#1080 todo change to 1080 for bluerobotics cameras
    camera_setup='mono' #'mono'

cam_res_rgbx,cam_res_rgby=cam_resx//2,cam_resy//2
fps=5
send_modulo=1 # Sends frames through gstreamer every x frames at above fps
save_modulo=3 # saves img every x frames at above fps

#gstreamer
gst_ports=[6760,6761]
gst_bitrate=1024*3
#gst_bitrate=256
gst_speed_preset=1

#tracker type
#tracker = 'local'
tracker = 'rope'

joy_deadband=0.00

thruster_limit=0.7

viewer_blacks=(50,100)


if __name__=='__main__':
    #for bash scripts to get state
    import sys
    print(globals()[sys.argv[1]],end='')
