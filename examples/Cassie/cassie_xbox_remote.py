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
        self.font = pygame.font.Font(None, 30)

    def print(self, screen, textString):
        textList = textString.split("\n")
        for string in textList:
            textBitmap = self.font.render(string, True, white)
            screen.blit(textBitmap, [self.x, self.y])
            self.y += self.line_height
        self.y += self.new_line_height


    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 30
        self.new_line_height = 40

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


def wait_for_joysticks(textPrint, screen, clock):
    while (pygame.joystick.get_count() != 1):
        screen.fill(cassie_blue)
        textPrint.reset()
        textPrint.print(screen, "please connect exactly one\ncontroller to use xbox remote feature")
        pygame.display.flip()
        clock.tick(20)

def main():
    publisher = lcm.LCM()

    pygame.display.set_caption('Cassie Virtual Radio Controller')

    pygame.init()
    screen_size = 500
    screen = pygame.display.set_mode((screen_size, screen_size))

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()
    textPrint = TextPrint()

    # radio spoof variables
    radio_channel_6_pos = 0
    radio_channel_6_delta = 0.05

    wait_for_joysticks(textPrint, screen, clock)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    done = False

    while not done:
        # DRAWING STEP
        # First, clear the screen to blue. Don't put other drawing commands
        # above this, or they will be erased with this command.
        screen.fill(cassie_blue)
        textPrint.reset()

        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.print(screen, "Welcome! remember to make this the active \nwindow when you wish to use the remote")
        textPrint.print(screen, "Controller detected: {}".format(name) )

        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                wait_for_joysticks(textPrint, screen, clock)

                joystick = pygame.joystick.Joystick(0)
                joystick.init()

            if event.type == pygame.QUIT: # If user clicked close
                done=True # Flag that we are done so we exit this loop

            if event.type == pygame.JOYHATMOTION:
                hat_val = joystick.get_hat(0)
                radio_channel_6_pos += radio_channel_6_delta * hat_val[1]
                # saturate between -1 and 1
                radio_channel_6_pos = min(max(radio_channel_6_pos, -1), 1)


        textPrint.print(screen, "Side dial position: {:.2f}".format(radio_channel_6_pos))

        # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()


        # Send LCM message
        radio_msg = dairlib.lcmt_radio_out()
        radio_msg.channel[0] = -joystick.get_axis(1)
        radio_msg.channel[1] = joystick.get_axis(0)
        radio_msg.channel[2] = -joystick.get_axis(4)
        radio_msg.channel[3] = joystick.get_axis(3)
        radio_msg.channel[6] = radio_channel_6_pos


        publisher.publish("CASSIE_VIRTUAL_RADIO", radio_msg.encode())

        # Limit to 20 frames per second
        clock.tick(20)

    pygame.quit()

if __name__ == '__main__':
    main()
