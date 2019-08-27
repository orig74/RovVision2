#cameras info
cam_resx,cam_resy=1920,1200
cam_res_rgbx,cam_res_rgby=cam_resx//2,cam_resy//2
reverse_camera_order=True
fps=10

#gstreamer
gst_ports=[6760,6761]
gst_bitrate=1024*3
#gst_bitrate=256
gst_speed_preset=1

#tracker type
#tracker = 'local'
tracker = 'rope'

joy_deadband=0.05

thruster_limit=0.5
