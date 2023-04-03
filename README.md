# systems_2022_23
SCR 2022-23's general systems; General range components used in different rover modules

 step 1: installed xacro and joint state publisher GUI
 	sudo apt install ros-humble-xacro (already installed)
 	sudo apt install ros-humble-joint-state-publisher-gui

step 2: installed gazebo ignition for Humble. Followed these instructions: https://github.com/gazebosim/ros_gz/tree/humble  
 	make sure to install with:
 	sudo apt install ros-humble-ros-ign
 	
 test with: ign gazebo
 run with: ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="src/systems_2022_23/helios/worlds/my_world.sdf"
 	
 step 3 ran rover URDF with following commands: (in ros workspace directory)
 	ros2 launch helios rsp.launch.py (to publish URDF)
 	ros2 run joint_state_publisher_gui joint_state_publisher_gui
 	rviz2 -d src/helios/config/view_bot.rviz (to run ROS visualizer)
 	
 	once you have this running, close joint_state_publisher
 	
Log 3: setting up ros to gazebo bridge: 
install gazebo bridge
	sudo apt-get install ros-humble-ros-ign-bridge
	
FOR DEBUG ONLY convert from xacro/urdf to sdf
convert xacro to urdf: xacro src/helios/description/robot.urdf.xacro > src/helios/description/robot.urdf
convert urdf to sdf: ign sdf -p src/helios/description/robot.urdf > src/helios/description/robot.sdf
load sdf into gazebo: ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "src/helios/description/robot.sdf", name: "helios"'

Log 4: launch sim with rover with:
	ros2 launch helios launch_sim.launch.py

Log 4: 
install ros2_control:
	sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

build ros_gz_control from source:


