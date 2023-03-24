# ========================================================================================
# Author:   Jasper Doan     @wluxie
# Date:     02/01/2023
# ========================================================================================
# Description:
#   This is the ROS2 node that publishes the controller input to the ROS2 topic 'controls'
#       The topic is a Float64MultiArray with 2 elements (in a [1x2] matrix, but stored as List)
#           ___                 ___
#          | l_analog    r_analog  |
#          |___                 ___|
#   Note:
#       The values are normalized to be between -1 and 1, deadzoned to be 0 if the value is 
#       less than AXIS_DEADZONE, and are rounded to 2 decimal places
#       The values are published at a rate of 10 Hz
# ========================================================================================
# Dependencies:
#   - ROS2 Foxy
#   - Python 3.8
#   - rclpy
# ========================================================================================


import rclpy                                # ROS2 Python API
from rclpy.node import Node                 # ROS2 Node API
from . import gamepad_input as gmi          # Gamepad input API by Cameron Rosenthal
from .buttons import Buttons                # Gamepad button callbacks
from std_msgs.msg import Float64MultiArray  # ROS2 Float64MultiArray message type



AXIS_DEADZONE = 0.1                                             # Deadzone is 0 to 1 | Note: axis value will be 0 until you move past the deadzone

b = Buttons()                                                   # Create button callbacks object
connectionEvents = [b.onGamepadConnect, b.onGamepadDisconnect]  # Set connection callbacks



# ==== ROS2 Publisher Node ===============================================================
# Brief Description:
#   When run, this node will take in your analog/joystick controller input and publish it to
#   the ROS2 topic 'controls' as a Float64MultiArray. We used PS4 during development, but
#   this should work with any controller that has a left stick, right stick, and 2 triggers.
#
# Publish:
#   - msg :: Float64MultiArray[2]
#      + msg.data[0] :: l_analog          + msg.data[1] :: r_analog
#
# Run in Terminal:
#   source /opt/ros/foxy/setup.bash             <------ Source ROS2 Foxy environment (if not already sourced)
#   cd ~/drive_2022_23/src/driver/              <------ Navigate to driver package directory
#
#       colcon build --symlink-install          <------ Build driver package (if not already built)
#       . install/setup.bash                    <------ Source driver package environment
#       ros2 run driver controller_pub          <------ Run controller_pub node
#
#       - or - 
#
#       make                                    <------ Make file that builts for you
#       make pub                                <------ Make file that sources and runs for you
# ========================================================================================
class ControllerPub(Node):
    # CONSTANTS
    TIMER_PERIOD = 0.1

    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Constructor
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, 
    def __init__(self):
        super().__init__('controller_pub')                                              # Create node with name 'controller_pub'

        # Publishing                                [type]                [topic]     [queue_size]
        self.publisher_ = self.create_publisher(Float64MultiArray, 'drive/analog_control', 10)      # Create publisher to publish controller input                                                          # Timer period in seconds
        self.timer = self.create_timer(ControllerPub.TIMER_PERIOD, self.timer_callback)             # Create timer to publish controller input

        # Note: I disabled button callbacks because they were causing the program to crash
        #       I think it was because the callbacks were trying to access the ROS2 node
        #       while the node was being spinned. I'm not sure, but I'll look into it
        #       later. For now, we can just use the controller input to drive the rover
        gmi.run_event_loop(None, None, None, connectionEvents)                          # Async loop to handle gamepad button events


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Timer Callback
    #       This function is called every time the timer goes off. It gets the controller
    #       input and publishes it to the ROS2 topic 'controls'
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    def timer_callback(self):
        msg = Float64MultiArray()                           # Create message

        gp = gmi.getGamepad(0)
        ls_x, ls_y = gmi.getLeftStick(gp, AXIS_DEADZONE)  # Get left stick
        rs_x, rs_y = gmi.getRightStick(gp, AXIS_DEADZONE) # Get right stick
        l2, r2 = gmi.getTriggers(gp, AXIS_DEADZONE)       # Get triggers
        l1, r1 = gmi.getButtonValue(gp, 7), gmi.getButtonValue(gp, 8)
    
        enable_left = l2 > 0
        enable_right = r2 > 0

        l_analog, r_analog = 0.0, 0.0

        if enable_left:
            l_analog = float(-ls_y)
        if enable_right:
            r_analog = float(-rs_y)

        if l1 or r1:
            r_analog = l_analog

        msg.data = [l_analog, r_analog]
        print('== SENDING [LS: (%.2f, %.2f) 😤 RS: (%.2f, %.2f)] ==' % (ls_x, ls_y, rs_x, rs_y))
        print('-- SENDING [L_ANALOG: %.2f 😤 R_ANALOG: %.2f] --' % (l_analog, r_analog))
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    controller_pub = ControllerPub()

    rclpy.spin(controller_pub)

    controller_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()