# hydrone_aerial_underwater_gazebo
# Dependencies

Messages:
mavros_msgs
geographic_msgs
trajectory_msgs
sensor_msgs
cv_bridge

Packages:
base_detector : https://github.com/ricardoGrando/base_detector

mav_comm: https://github.com/ethz-asl/mav_comm

rotors_simulator: https://github.com/ethz-asl/rotors_simulator.git

To run:

roslaunch hydrone_aerial_gazebo hydrone_aerial.launch

rostopic pub /mavros/state mavros_msgs/State "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
connected: false
armed: false
guided: false
manual_input: false
mode: 'ACRO'
system_status: 0" 

Installation of the Arena:

Copy the models of hydrone_aerial_gazebo/models to ~/.gazebo/models

