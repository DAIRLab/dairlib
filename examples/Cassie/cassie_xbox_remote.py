import sys
import pygame
import dairlib.lcmt_radio_out
import lcm
import numpy as np

# colors
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


def main():
    ramp_speed = len(sys.argv) > 1
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

    if (pygame.joystick.get_count() != 1):
        raise RuntimeError("Please connect exactly one controller")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    done = False
    max_speed = 1.0/3.0
    ramp_up = np.arange(0, max_speed, 0.01)
    stay = max_speed * np.ones(400)
    ramp_down = np.flip(np.arange(0, max_speed, 0.01))
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
        textPrint.print(screen, "Welcome! remember to make this the active \nwindow when you wish to use the remote")
        textPrint.print(screen, "Controller detected: {}".format(name) )

        for event in pygame.event.get():
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
