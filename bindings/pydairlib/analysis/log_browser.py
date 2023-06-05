"""
    PyGame utility to support playing back LCM logs in a more streamlined
    fashion than LcmLogPlayer by using the arrow keys to navigate between logs.
"""
import os
import lcm
import glob
import pygame
import argparse

from typing import List


class LcmLogPlayer:
    def __init__(self, filename: str, channels: List[str]):
        self.lcm_log = lcm.EventLog(filename, "r")
        self.channels = channels
        self.lc = lcm.LCM()

    def get_next_message(self):
        assert self.channels

        event = self.lcm_log.read_next_event()
        while event.channel not in self.channels:
            event = self.lcm_log.read_next_event()
        return event.channel, event.data

    def publish_next(self):
        channel, data = self.get_next_message()
        # TODO (@Brian-Acosta) does this need to be decoded?
        self.lc.publish(channel, data)

    def reset(self):
        self.lcm_log.seek(0)


def get_log_names(logfolder, logtemplate):
    logpath_template = os.path.join(logfolder, logtemplate)
    logs = glob.glob(f'{logpath_template}[0-9][0-9]')
    print("Found logs:")
    for log in logs:
        print(log)
    return logs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log_folder",
        nargs=1,
        default="",
        help="Folder containing lcm logs",
        type=str,
        required=True
    )
    parser.add_argument(
        "--log_name_template",
        nargs=1,
        default="lcmlog-",
        help="Template for lcm log name - all lcmlogs should be named "
             "<template>-##, where ## is the log number",
        type=str
    )
    parser.add_argument(
        "--channels",
        nargs="+",
        default=[""],
        help="List of lcm channels to play back"
    )
    args = parser.parse_args()

    # PyGame Setup
    pygame.init()
    screen_width = 800
    screen_height = 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Lcm Log Browser")
    font = pygame.font.Font(None, 36)

    running = True
    playing = False

    assert(os.path.isdir(args.log_folder))

    logpaths = get_log_names(args.log_folder, args.log_name_template)
    active_log_idx = 0
    nlogs = len(logpaths)
    logplayers = {}

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Check for keyboard events
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    playing = not playing  # Toggle play/pause state

                elif event.key == pygame.K_RIGHT:
                    active_log_idx = (active_log_idx + 1) % nlogs

                elif event.key == pygame.K_LEFT:
                    active_log_idx -= 1
                    active_log_idx = active_log_idx if active_log_idx >= 0 \
                        else nlogs - 1

        if active_log_idx not in logplayers:
            logplayers[active_log_idx] = LcmLogPlayer(
                logpaths[active_log_idx],
                args.channels
            )

        if playing:
            logplayers[active_log_idx].publish_next()

        # Clear the screen
        screen.fill((0, 0, 0))

        # Render the current stream name
        text_surface = font.render(logpaths[active_log_idx], True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=(screen_width // 2, screen_height // 2))
        screen.blit(text_surface, text_rect)

        # Update the display
        pygame.display.flip()

    # Quit Pygame
    pygame.quit()


if __name__ == '__main__':
    main()
