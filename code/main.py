from kame import Kame

robot = Kame([-5, -14, 4, 20, -10, 0, 12, 5], [3, 2, 15, 12, 1, 0,  14, 13])

# robot.zero()
# robot.jump()
# robot.turnR(4)
robot.walk(14)
robot.turnR(6)
robot.walk(8)
robot.jump()
robot.jump()
robot.turnL(6)
robot.walk(8)
robot.turnL(6)
robot.walk(8)
robot.turnR(6)
robot.walk(8)


# robot.turnL(4)
# robot.back(20)
