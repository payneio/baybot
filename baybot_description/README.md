# Baybot Description

```bash

# Launch in rviz
roslaunch baybot_description rviz.launch

# Launch in gazebo
roslaunch gazebo_ros empty_world.launch
roslaunch baybot_description spawn.launch

# Teleop in gazebo
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel

# Delete in gazebo
rosservice call /gazebo/delete_model "model_name: 'baybot'"

# Add obstacle
rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/object.urdf -urdf -x 1 -y 0 -z 1 -model my_object

```