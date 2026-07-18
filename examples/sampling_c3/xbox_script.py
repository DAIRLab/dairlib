"""Teleoperation controls for sampling C3 controller using an xbox controller.

Button presses:
- A: Toggle teleoperation mode
- X: Toggle force tracking mode
- Y: Toggle force C3 mode
- Start: Print cost breakdown
- Back: Print current rotation and position cost

Mapping to radio channels:
- Channel 0: x-direction teleop
- Channel 1: y-direction teleop
- Channel 2: z-direction teleop
- Channel 6: Back button (print current rotation and position cost)
- Channel 7: Start button (print cost breakdown)
- Channel 11: X button (force tracking mode)
- Channel 12: Y button (force C3 mode)
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

    if (pygame.joystick.get_count() != 1):
        raise RuntimeError("Please connect exactly one controller")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    done = False
    i = 0
    latching_switch_a = 1
    latching_switch_x = 0
    latching_switch_y = 0
    latching_switch_start = 0
    latching_switch_back = 0
    print("Teleop Status: " + str(latching_switch_a))
    print("Force Tracking Status: " + str(not latching_switch_x))


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
                if event.button == 2:
                    latching_switch_x = not latching_switch_x
                    print("Force Tracking Status: " + str(not latching_switch_x))
                if event.button == 3:
                    latching_switch_y = not latching_switch_y
                    print("Force C3 Mode Status: " + str(latching_switch_y))
                if event.button == 7:
                    latching_switch_start = not latching_switch_start  
                    print("Print cost breakdown status: " + str(latching_switch_start))
                if event.button == 6:
                    latching_switch_back = not latching_switch_back  
                    print("Print current rot and pos cost status: " + str(latching_switch_back))

        # Send LCM message
        radio_msg = dairlib.lcmt_radio_out()
        radio_msg.channel[0] = -joystick.get_axis(1)
        radio_msg.channel[1] = -joystick.get_axis(0)
        radio_msg.channel[2] = -joystick.get_axis(4)
        radio_msg.channel[6] = latching_switch_back
        radio_msg.channel[7] = latching_switch_start
        radio_msg.channel[11] = latching_switch_x
        radio_msg.channel[12] = latching_switch_y
        radio_msg.channel[14] = latching_switch_a

        publisher.publish("SAMPLING_C3_RADIO", radio_msg.encode())

        # Limit to 20 frames per second
        clock.tick(60)
        i += 1

    pygame.quit()

if __name__ == '__main__':
    main()
