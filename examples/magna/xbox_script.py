"""Mapping button A to switch between teleoperation and MPC mode.
We use SpaceMouse for teleoperation, so other buttons of the xbox controller are not used.

Button presses:
- A: Toggle teleoperation mode

Mapping to radio channels:
- Channel 14: A button (teleoperation mode)
"""

import pygame
import dairlib.lcmt_radio_out
import lcm
import numpy as np


def main():
    publisher = lcm.LCM()

    pygame.init()

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()

    if pygame.joystick.get_count() != 1:
        raise RuntimeError("Please connect exactly one controller")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    done = False
    i = 0
    latching_switch_a = True
    print("Teleop Status: " + str(latching_switch_a))

    while not done:
        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                # use this print statement to find the button number
                # print(f"Button {event.button} pressed")
                if event.button == 0:
                    latching_switch_a = not latching_switch_a
                    print("Teleop Status: " + str(latching_switch_a))

        # Send LCM message
        radio_msg = dairlib.lcmt_radio_out()
        radio_msg.channel[14] = latching_switch_a

        publisher.publish("MAGNA_RADIO", radio_msg.encode())

        # Limit to 20 frames per second
        clock.tick(60)
        i += 1

    pygame.quit()


if __name__ == "__main__":
    main()
