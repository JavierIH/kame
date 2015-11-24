import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import pygame
import time
from kame import Kame
robot = Kame([-5, -14, 4, 20, -10, 0, 12, 5], [3, 2, 15, 12, 1, 0,  14, 13])

pygame.init()
clock = pygame.time.Clock()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
T = 320.0

while True:
    for event in pygame.event.get():
        pass

#    axis = joystick.get_axis(3)
#    T = (axis + 1)*1000 + 100


    button = joystick.get_button(2)

    hat = joystick.get_hat(0)
#    print 'hat[0]: ', hat[0], 'hat[1]: ', hat[1]

    if button == 1:
        robot.jump()

    elif hat[1] == 1:
        print 'walk'
        robot.walk(1, T)

    elif hat[1] == -1:
        print 'back'
        robot.back(1, T)

    elif hat[0] == 1:
        print 'turnR'
        robot.turnR(1, T)

    elif hat[0] == -1:
        print 'turnL'
        robot.turnL(1, T)

    else:
        robot.zero()

    clock.tick(50)

pygame.quit()
