# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
topic_sitl_position_report=b'position_rep'
topic_sitl_position_report_port=7755

topic_thrusters_comand=b'thruster_cmd'
#opration modes
topic_system_state=b'system_state'

topic_controller_port=topic_thrusters_comand_port=7788
thrusters_sink_port = 7787
topic_lights=b'topic_lights'


#topic_camera_left=b'topic_camera_left'
#topic_camera_right=b'topic_camera_right'
topic_stereo_camera=b'topic_stereo_camera'
topic_stereo_camera_ts=b'topic_stereo_camera_ts'
topic_camera_port=7789

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

#messages:
#stop/start recording

topic_record_state=b'record_state'
topic_record_state_port=9303

topic_local_route_port=9995

topic_viewer_data=b'topic_viewer_data'

topic_depth_hold_pid=b'topic_depth_control'
topic_depth_hold_port=9996

topic_tracker=b'topic_tracker'
topic_tracker_port=10101

topic_volt=b'topic_volt'
topic_volt_port=10102
