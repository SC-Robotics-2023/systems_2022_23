# systems_2022_23
## SCR 2022-23's general systems; General range components used in different rover modules

Install dependencies and tools using `systems_setup.bash`

> Make sure to source your overlay before running any commands
	source install/setup.bash

> make sure to run all commands from your ROS/colcon workspace

### Run Rviz
	rviz2 -d src/systems_2022_23/helios/config/rover_view.rviz

### Visualize URDF Rover
	ros2 launch helios visualize.launch.py

### Run Simulation (robot state publisher + gazebo + rviz)
	ros2 launch helios gazebo_sim.launch.py
	
> if you want to see the rover move in rviz, change 
Fixed Frame from base_link to odom

### Run keyboard controller
	ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

### Run gamepad controller
	ros2 run driver gamepad_diff_drive

> make sure to have drive repo cloned and built
