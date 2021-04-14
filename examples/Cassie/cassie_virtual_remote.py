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

import dairlib.lcmt_cassie_out

import time
import lcm


class KeyboardManager():

    def __init__(self):
        pygame.init()
        screen_size = 400
        self.screen = pygame.display.set_mode((screen_size, screen_size))
        self.vel = np.array([0.0, 0.0])
        self.trim_x = 0
        self.trim_y = 0
        self.delta_vx = 0.1
        self.delta_vy = 0.05
        self.delta_z = 0.05
        self.height_adjust = 0


    def switch_motion_key(self, key):
        switcher = {
            K_w: np.array([self.delta_vx, 0.0]),
            K_s: np.array([-self.delta_vx, 0.0]),
            K_a: np.array([0.0, self.delta_vy]),
            K_d: np.array([0.0, -self.delta_vy])
        }
        return switcher.get(key, np.array([0, 0]))

    def event_callback(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN:
                if (event.key == K_u):
                    self.height_adjust += self.delta_z
                    return
                elif (event.key == K_d):
                    self.height_adjust -= self.delta_z
                    return

                self.vel += self.switch_motion_key(event.key)
                if np.abs(self.vel[0]) < 1e-2:
                    self.vel[0] = 0.0
                if np.abs(self.vel[1]) < 1e-2:
                    self.vel[1] = 0.0


def main():
    keyboard = KeyboardManager()

    lc = lcm.LCM()

    cassie_blue = (6, 61, 128)
    white = (255, 255, 255)
    pygame.display.set_caption('Cassie Virtual Radio Controller')
    keyboard.screen.fill(cassie_blue)
    fnt = pygame.font.Font('freesansbold.ttf', 32)

    while True:
        # Get any keypresses
        keyboard.event_callback()

        # Update display screen
        display_text1 = 'Sagittal vel: ' + '%.2f'%(keyboard.vel[0])
        display_text2 = 'Coronal Vel: ' + '%.2f'%(keyboard.vel[1])
        text1 = fnt.render(display_text1, True, white, cassie_blue)
        text2 = fnt.render(display_text2, True, white, cassie_blue)
        text_rect1 = text1.get_rect()
        text_rect2 = text2.get_rect()
        text_rect1.center = (200, 150)
        text_rect2.center = (200, 250)
        keyboard.screen.fill(cassie_blue)
        keyboard.screen.blit(text1, text_rect1)
        keyboard.screen.blit(text2, text_rect2)

        # Send LCM message
        cassie_out_msg = dairlib.lcmt_cassie_out()
        cassie_out_msg.pelvis.radio.channel[0] = keyboard.vel[0] + keyboard.trim_x
        cassie_out_msg.pelvis.radio.channel[1] = keyboard.vel[1] + keyboard.trim_y
        cassie_out_msg.pelvis.radio.channel[3] = 0

        lc.publish("CASSIE_OUTPUT_ECHO", cassie_out_msg.encode())

        time.sleep(0.05)
        pygame.display.update()

if __name__ == '__main__':
    main()
