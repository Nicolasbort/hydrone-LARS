# Visual-based Autonomous Unmanned Aerial Vehicle for Inspection in Indoor Environments

## Installing the packages:

<pre>
mkdir -p ~/hydrone_workspace/src && cd ~/hydrone_workspace/src
git clone https://github.com/Nicolasbort/hydrone-LARS.git
cd ..
catkin_make
</pre>


## Installing the gazebo models:


Move the models from `~/hydrone_workspace/src/hydrone-LARS/hydrone_aerial_gazebo/models` to `~/.gazebo/models`


## How to run the simulation environment:

`source ~/hydrone_workspace/devel/setup.bash`

* Mapping task:

`roslaunch hydrone_aerial_gazebo hydrone_aerial_mapping.launch`

* Sensing task:

`roslaunch hydrone_aerial_gazebo hydrone_aerial_sensoring.launch`

* Inspection task:

`roslaunch hydrone_aerial_gazebo hydrone_aerial_inspection.launch`

## How to start the UAV:

<pre>
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
</pre>

