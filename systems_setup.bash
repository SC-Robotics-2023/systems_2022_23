sudo apt update && sudo apt upgrade -y

# Install Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs -y

# Install xacro and joint state publisher GUI
sudo apt install ros-humble-xacro -y
sudo apt install ros-humble-joint-state-publisher-gui -y

# Install ros2 control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control -y





# Install any missing dependencies
rosdep install --from-paths src --ignore-src -r -y