# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
topic_sitl_position_report=b'position_rep'
topic_sitl_position_report_port=7755

topic_thrusters_comand=b'thruster_cmd'
topic_gripper_cmd=b'topic_gripper_cmd'
#opration modes
topic_system_state=b'system_state'

topic_controller_port=topic_thrusters_comand_port=7788
thrusters_sink_port = 7787
printer_sink_port = 7445
topic_lights=b'topic_lights'
topic_camera_servo=b'camera_servo'


#topic_camera_left=b'topic_camera_left'
#topic_camera_right=b'topic_camera_right'
topic_stereo_camera=b'topic_stereo_camera'
topic_stereo_camera_ts=b'topic_stereo_camera_ts'

topic_main_cam=b'topic_main_cam'
topic_main_cam_depth=b'topic_main_cam_depth'

topic_camera_telem=b'topic_camera_telem'
topic_camera_port=7789
topic_camera_ts_port=17789
topic_camera_telem_port=17790

topic_stereo_camera_calib=b'topic_stereo_camera_calib'
topic_camera_calib_port=8890

topic_button = b'joy_button'
topic_axes = b'joy_axes'
topic_hat = b'joy_hat'
topic_joy_port=8899

#diffrent topics due to difrent freq devices
topic_imu = b'topic_imu'
topic_imu_port = 8897

topic_sonar = b'topic_sonar'
topic_sonar_port = 9301

topic_depth = b'topic_depth'
topic_depth_port = 9302

topic_gps=b'topic_gps'
topic_gps_port=9304

#messages:
#stop/start recording

topic_record_state=b'record_state'
topic_record_state_port=9303

topic_local_route_port=9995

topic_viewer_data=b'topic_viewer_data'

topic_depth_hold_pid=b'topic_depth_control'
topic_depth_hold_port=9996
topic_sonar_hold_pid=b'topic_sonar_control'
topic_sonar_hold_port=9998

topic_att_hold_yaw_pid=b'topic_att_yaw_control'
topic_att_hold_pitch_pid=b'topic_att_pitch_control'
topic_att_hold_roll_pid=b'topic_att_roll_control'
topic_att_hold_port=10052

topic_pos_hold_pid_fmt=b'topic_pos_hold_pid_%d'
topic_pos_hold_port=10053


topic_tracker=b'topic_tracker'
topic_tracker_port=10101

topic_telem=b'topic_hw_telem'
topic_telem_port=10102

topic_hw_stats=b'topic_hw_stats'
topic_hw_stats_port=10103

topic_dvl_raw=b'topic_dvl_raw'
topic_dvl_vel = b'topic_vel'
topic_dvl_port=13295

topic_dvl_cmd=b'topic_dvl_cmd'

topic_remote_cmd=b'topic_remote_cmd'
topic_remote_cmd_port=13297


topic_volt=b'topic_volt'
