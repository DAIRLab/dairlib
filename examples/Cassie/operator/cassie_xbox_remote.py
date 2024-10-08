import sys
import pygame
import dairlib.lcmt_radio_out
import lcm
import numpy as np
from examples.Cassie.operator.pygame_utils import TextPrint

# colors
cassie_blue = (6, 61, 128)
white = (255, 255, 255)

def main():
    ramp_speed = len(sys.argv) > 1
    publisher = lcm.LCM()

    pygame.display.set_caption('Cassie Virtual Radio Controller')

    pygame.init()
    screen_size = 500
    screen = pygame.display.set_mode((screen_size, screen_size))

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()
    textPrint = TextPrint(30, (10, 10))

    # radio spoof variables
    radio_channel_6_pos = 0
    radio_channel_6_delta = 0.05

    pygame.joystick.init()
    if (pygame.joystick.get_count() != 1):
        raise RuntimeError("Please connect exactly one controller")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    done = False
    max_speed = 0.25
    ramp_up = np.arange(0, max_speed, 0.01)
    stay = max_speed * np.ones(120)
    ramp_down = np.flip(np.arange(-0.05, max_speed, 0.02))
    speeds = np.hstack((ramp_up, stay, ramp_down))
    n = speeds.size
    i = 0
    while not done:
        # DRAWING STEP
        # First, clear the screen to blue. Don't put other drawing commands
        # above this, or they will be erased with this command.
        screen.fill(cassie_blue)
        textPrint.reset()

        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.print(
            screen,
            "Welcome! remember to make this the active "
            "\nwindow when you wish to use the remote",
            white
        )
        textPrint.print(screen, "Controller detected: {}".format(name), white)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: # If user clicked close
                done=True # Flag that we are done so we exit this loop

            if event.type == pygame.JOYHATMOTION:
                hat_val = joystick.get_hat(0)
                radio_channel_6_pos += radio_channel_6_delta * hat_val[1]
                # saturate between -1 and 1
                radio_channel_6_pos = min(max(radio_channel_6_pos, -1), 1)

        textPrint.print(
            screen, "Side dial position: {:.2f}".format(
                radio_channel_6_pos
            ), white
        )

        # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()


        # Send LCM message
        radio_msg = dairlib.lcmt_radio_out()
        if ramp_speed and i < n:
            radio_msg.channel[0] = speeds[i]
        else:
            radio_msg.channel[0] = -joystick.get_axis(1)
        radio_msg.channel[1] = joystick.get_axis(0)
        radio_msg.channel[2] = -joystick.get_axis(4)
        radio_msg.channel[3] = joystick.get_axis(3)
        radio_msg.channel[6] = radio_channel_6_pos

        radio_msg.channel[15] = -1 * np.rint(joystick.get_axis(5))

        publisher.publish("CASSIE_VIRTUAL_RADIO", radio_msg.encode())

        # Limit to 20 frames per second
        clock.tick(20)
        i += 1

    pygame.quit()


if __name__ == '__main__':
    main()
