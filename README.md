# systems_2022_23
SCR 2022-23's general systems; General range components used in different rover modules

Install dependencies and tools using systems_setup.bash

Visualize URDF Rover
	(from ros2_ws)
	Terminal 1: ros2 launch helios rsp.launch.py
	Terminal 2: rviz2 -d src/systems_2022_23/helios/config/view_bot.rviz
	Terminal 3: ros2 run joint_state_publisher_gui joint_state_publisher_gui

Run Simulation
	(from ros2_ws)
	Terminal 1: ros2 launch helios gazebo_sim.launch.py
	Terminal 2: rviz2 -d src/systems_2022_23/helios/config/view_bot.rviz