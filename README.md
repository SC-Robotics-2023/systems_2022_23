# systems_2022_23
SCR 2022-23's general systems; General range components used in different rover modules

Log 1: installed gazebo for Humble. Followed these instructions: 
	 sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
 	sudo apt-get update
 	sudo apt install ros-humble-ros-ign
 	
 test with: ign gazebo
 
 Log 2: installed xacro and joint state publisher GUI
 	sudo apt install ros-humble-xacro (already installed)
 	sudo apt install ros-humble-joint-state-publisher-gui
