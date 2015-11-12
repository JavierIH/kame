import time
from kame import Kame

x = 10
robot = Kame([3, -8, 5, 14, -12, -5, 20, 1], [0, 1, 13, 14, 2, 3, 12, 15])
#robot.zero()
robot.walk2(100)

#while True:
#    robot.jump()
#    time.sleep(0.065)
