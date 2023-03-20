import dairlib.lcmt_radio_out
import pygame
import lcm
import numpy as np

# colors
cassie_blue = (6, 61, 128)
white = (255, 255, 255)

def main():
    publisher = lcm.LCM()

    screen_size = 500
    # screen = pygame.display.set_mode((screen_size, screen_size))

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()

    done = False
    max_speed = 2.8
    dt = .020
    acceleration = 2.0 # m/s^2
    deceleration = 0.5 # m/s^2
    max_speed_duration = 9
    velocity_step_up = acceleration * dt
    velocity_step_down = deceleration * dt
    ramp_up = np.arange(0, max_speed, velocity_step_up)
    stay = max_speed * np.ones(int(max_speed_duration / dt))
    ramp_down = np.flip(np.arange(0, max_speed, velocity_step_down))
    speeds = np.hstack((ramp_up, stay, ramp_down))
    ramp_duration = speeds.shape[0] * .02
    print("total duration: ", ramp_duration)
    i = 0
    while not done:
        # DRAWING STEP
        # First, clear the screen to blue. Don't put other drawing commands
        # above this, or they will be erased with this command.
        # screen.fill(cassie_blue)

        # Send LCM message
        radio_msg = dairlib.lcmt_radio_out()
        radio_msg.channel[0] = speeds[i]

        publisher.publish("CASSIE_VIRTUAL_RADIO", radio_msg.encode())

        # Limit to 20 frames per second
        clock.tick(dt * 1000)
        i += 1

    pygame.quit()

if __name__ == '__main__':
    main()
