import numpy as np
#track_offx=0
track_offx=80
#track_params = (60,60,60,60,track_offx,0)
track_params = (150,150,150,150,track_offx,0)
stereo_corr_params = {'ws':(128,128),'sxl':250+50+200,'sxr':0,'ofx':100 ,'sxl2':100, 'sxr2':100}
fov=60.97
diff_range_valid=1.0
clear_freqs=5
max_diff_cols=50
#max_diff_cols=150
#max_diff_cols=1500
ignore_extrema_type=False

pixelwidthx = 2464//4
pixelwidthy = 2056//4  
baseline = 0.065 # (240-100)*.1scale in cm from unreal engine
focal_length=pixelwidthx/( np.tan(np.deg2rad(fov/2)) *2 )
camera_pitch=0

