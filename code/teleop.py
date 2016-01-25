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


init_ref = time.time()
g_phase = 0
T = 350.0

while True:
    for event in pygame.event.get():
        pass

    axis_ax = joystick.get_axis(0)
    axis_ay = joystick.get_axis(1)
    inclination = [joystick.get_axis(2), joystick.get_axis(3)]
    cross = joystick.get_hat(0)
    a_button = joystick.get_button(2)
    left_trigger = joystick.get_button(6)
    right_trigger = joystick.get_button(7)


    if axis_ax == 0 and axis_ay == 0:
        g_phase = 0

        if a_button == 1:
            robot.jump()

        elif cross[1] == 1:
            print 'walk'
            robot.walk(1, T)

        elif cross[1] == -1:
            print 'back'
            robot.back(1, T)

        elif cross[0] == 1:
            print 'turnR'
            robot.turnR(1, T)

        elif cross[0] == -1:
            print 'turnL'
            robot.turnL(1, T)
        else:
            robot.home()

    else:
        g_phase += robot.omnimove(g_phase, inclination, axis_ax, axis_ay)

    clock.tick(200)

pygame.quit()
