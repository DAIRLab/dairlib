'''
Uses Pygame (pip install pygame) to capture keyboard input. The virtual remote must be the active window to register commands.
W / S to increase/decrease sagittal velocity
A / D to increase/decrease longitudinal velocity

use Keyboard Manager trim_x and trim_y member variables to add trim to the velocity commands
'''

import sys
import pygame
from pygame.locals import *
import numpy as np

import dairlib.lcmt_radio_out

import time
import lcm


cassie_blue = (6, 61, 128)
white = (255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def print(self, screen, textString):
        textBitmap = self.font.render(textString, True, white)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10



def main():
    publisher = lcm.LCM()

    pygame.display.set_caption('Cassie Virtual Radio Controller')

    pygame.init()
    screen_size = 400
    screen = pygame.display.set_mode((screen_size, screen_size))

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()
    textPrint = TextPrint()

    while True:
        # DRAWING STEP
        # First, clear the screen to white. Don't put other drawing commands
        # above this, or they will be erased with this command.
        screen.fill(cassie_blue)
        textPrint.reset()

        # Get count of joysticks
        if (pygame.joystick.get_count() != 1):
            raise RuntimeError('Please Connect Exactly One Controller')


        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.print(screen, "Joystick name: {}".format(name) )

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        textPrint.print(screen, "Number of axes: {}".format(axes) )
        textPrint.indent()


        # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

        # Limit to 10 frames per second
        clock.tick(20)

        screen.fill(cassie_blue)

        # Send LCM message
        radio_msg = dairlib.lcmt_radio_out()
        radio_msg.channel[0] = -joystick.get_axis(1)
        radio_msg.channel[1] = joystick.get_axis(0)
        radio_msg.channel[3] = joystick.get_axis(3)

        publisher.publish("CASSIE_VIRTUAL_RADIO", radio_msg.encode())

        time.sleep(0.05)
        pygame.display.update()

if __name__ == '__main__':
    main()
