import time
from kame import Kame

x = 10
robot = Kame([3, -8, 5, 16, -12, -5, 20, 4], [0, 1, 4, 5, 3, 2, 7, 6])
#robot.zero()
robot.run(100)

#while True:
#    robot.jump()
#    time.sleep(0.065)
