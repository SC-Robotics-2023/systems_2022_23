# systems_2022_23
SCR 2022-23's general systems; General range components used in different rover modules

Install dependencies and tools using systems_setup.bash

rviz2 -d src/systems_2022_23/helios/config/view_bot.rviz

Visualize URDF Rover
	(from ros2_ws)
	Terminal 1 (robot state publisher): ros2 launch helios rsp.launch.py
	Terminal 2 (robot visualizer): rviz2 -d src/systems_2022_23/helios/config/view_bot.rviz
	Terminal 3 (joint state emulator): ros2 run joint_state_publisher_gui joint_state_publisher_gui

Run Simulation and Control Rover
	(from ros2_ws)
	Terminal 1 (robot state publisher + simulation): ros2 launch helios gazebo_sim.launch.py
	Terminal 2 (robot visualizer): rviz2 -d src/systems_2022_23/helios/config/view_bot.rviz
	Terminal 3 (keyboard controller): ros2 run ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped


Run keyboard controller
	(from ros2_ws)

