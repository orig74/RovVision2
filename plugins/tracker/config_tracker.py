import numpy as np
track_offx=80
#track_params = (60,60,60,60,track_offx,0)
track_params = (80,80,80,80,track_offx,0)
stereo_corr_params = {'ws':(128,128),'sxl':250+50+200,'sxr':0,'ofx':80 ,'sxl2':40, 'sxr2':40}
fov=60.97
diff_range_valid=1.0
clear_freqs=15
max_diff_cols=10
pixelwidthx = 960
pixelwidthy = 600  
baseline = 0.122 # (240-100)*.1scale in cm from unreal engine
focal_length=pixelwidthx/( np.tan(np.deg2rad(fov/2)) *2 )
camera_pitch=0

