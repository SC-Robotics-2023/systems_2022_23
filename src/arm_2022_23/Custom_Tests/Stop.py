import time
from roboclaw_3 import Roboclaw

#Windows comport name
# rc = Roboclaw("COM11",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80

rc.ForwardM1(address,0)	#1/4 power forward