"""
    PyGame utility to support playing back LCM logs in a more streamlined
    fashion than LcmLogPlayer by using the arrow keys to navigate between logs.
"""

import pygame
import argparse

# Initialize Pygame
pygame.init()

# Set the screen size
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))

# Set the caption
pygame.display.set_caption("Lcm Log Browser")

# Set the initial state
is_playing = False

parser = argparse.ArgumentParser()
parser.add_argument("--log_folder", nargs=1, default="",
                    help="Folder containing lcm logs", type=str, required=True)
parser.add_argument("--log_name_template", nargs=1, default="lcmlog-",
                    help="Template for lcm log name - all lcmlogs should be "
                         "named <template>-##, where ## is the log number",
                    type=str)
parser.add_argument("--channels", nargs="+", default=[""],
                    help="List of lcm channels to play back")

args = parser.parse_args()

# Load the font
font = pygame.font.Font(None, 36)

# Helper function to play channels from the current stream
def play_log(logpath):
    # Add code to play the channels here

# Game loop
running = True
current_stream_name = "Stream 1"  # Default current stream
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Check for keyboard events
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                is_playing = not is_playing  # Toggle play/pause state
                if is_playing:
                    print("Stream started")
                    # Add code to start the stream here
                    play_channels(current_stream_name)
                else:
                    print("Stream paused")
                    # Add code to pause the stream here

            elif event.key == pygame.K_RIGHT:
                print("Next")
                # Add code to go to the next stream here
                # Update current_stream_name to the next stream

            elif event.key == pygame.K_LEFT:
                print("Back")
                # Add code to go back to the previous stream here
                # Update current_stream_name to the previous stream

    # Clear the screen
    screen.fill((0, 0, 0))

    # Render the current stream name
    text_surface = font.render(current_stream_name, True, (255, 255, 255))
    text_rect = text_surface.get_rect(center=(screen_width // 2, screen_height // 2))
    screen.blit(text_surface, text_rect)

    # Update the display
    pygame.display.flip()

# Quit Pygame
pygame.quit()
