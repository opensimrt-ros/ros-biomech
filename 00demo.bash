#!/usr/bin/env bash
SESSION_NAME=demo

source "`rospack find tmux_session_core`/common_functions.bash"
ros_core_tmux "$SESSION_NAME"

tmux set -g pane-border-status top

W1=(
#"rqt_plot"
#"rosrun flexbe_input behavior_input" ## included in flexbe_all
"rosrun acquisition_of_raw_data multiservice_plex.py"
"roslaunch acquisition_of_raw_data flexbe_all.launch"
"roslaunch test_moticon_insoles show_urdf_everything.launch use_gui:=false"
#"roslaunch ximu3_ros ximu_lower.launch do_calibration:=true parent_frame_id:=subject_heading wait_to_start:=true"
"roslaunch ximu3_ros ximu_lower.launch do_calibration:=true wait_to_start:=true"
"roslaunch osrt_ros t42.launch run_as_service:=true parent_frame:=map"
#"roslaunch osrt_ros ik_lowerbody_inverted_pelvis.launch"
"roslaunch osrt_ros t46.launch bypass_heading_computation:=true heading_debug:=0 visualise:=false orientation_server_type:=subscriber wait_to_start:=true"
##"rosrun rqt_robot_monitor rqt_robot_monitor" ## in the future, this is nice to organize things but it looks complicated
"rosrun rqt_runtime_monitor rqt_runtime_monitor"
"rosrun rqt_reconfigure rqt_reconfigure"
"roslaunch osrt_ros vis_ik.launch"
#"rosrun rqt_graph rqt_graph"
)

##TODO: about the rqt_robot_monitor. i started creating a package for it, it would show like the insoles and imus and everything eelse in the future in like an organized way so we could check stuff, not done though

W2=(
"roslaunch moticon_insoles read_sdk.launch tf_prefix:=ik/ --wait"
"roslaunch republisher republisher_insoles.launch --wait"
)

W3=(
"#rosservice call /ik_lowerbody_node/set_name_and_path \"{name: 'trialX', path: '/tmp/' }\" --wait" 
"#rosservice call /ik_lowerbody_node/start_recording"
"#rosservice call /ik_lowerbody_node/stop_recording"
"#rosservice call /ik_lowerbody_node/write_sto"
"#rosservice call /ik_lowerbody_node/clear_loggers"
)
W4=(
"#rosservice call /moticon_insoles/setfilename \"{name: 'trialX', path: '/tmp/' }\" --wait" 
"#rosservice call /ik_lowerbody_node/record"
"#rosservice call /ik_lowerbody_node/stop"
"#rosservice call /ik_lowerbody_node/save"
"#rosservice call /ik_lowerbody_node/clear"
""
)
W5=(
"#rosbag record /right/insole /left/insole /right/wrench /right/wrench_filtered /left/wrench /left/wrench_filtered /ximu_femur_l/imu /ximu_femur_r/imu /ximu_pelvis/imu /ximu_tibia_l/imu /ximu_tibia_r/imu /ximu_talus_l/imu /ximu_talus_r/imu -O /tmp/trialX"
#"rosrun rqt_bag rqt_bag"
)

create_tmux_window "$SESSION_NAME" "main_nodes" "${W1[@]}"
#create_tmux_window "$SESSION_NAME" "insole_nodes" "${W2[@]}"
#create_tmux_window "$SESSION_NAME" "ik_savers" "${W3[@]}"
#create_tmux_window "$SESSION_NAME" "insole_savers" "${W4[@]}"
#create_tmux_window "$SESSION_NAME" "rosbag" "${W5[@]}"

tmux -2 a -t $SESSION_NAME


