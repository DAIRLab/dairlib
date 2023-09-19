"""
    PyGame utility to support playing back LCM logs in a more streamlined
    fashion than LcmLogPlayer by using the arrow keys to navigate between logs.
"""
import os
import lcm
import sys
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
        while event and (event.channel not in self.channels):
            event = self.lcm_log.read_next_event()

        if not event:
            return None, None
        return event.channel, event.data

    def publish_next(self):
        channel, data = self.get_next_message()
        if channel and data:
            self.lc.publish(channel, data)
        else:
            self.reset()
        return channel is not None

    def reset(self):
        self.lcm_log.seek(0)


def get_log_names(logfolder, logtemplate):
    if isinstance(logtemplate, list):
        logtemplate = logtemplate[0]
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
        default="",
        help="List of lcm channels to play back"
    )
    args = parser.parse_args()
    log_folder = args.log_folder[0]
    log_name_template = args.log_name_template

    # PyGame Setup
    pygame.init()
    screen_width = 1500
    screen_height = 200
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Lcm Log Browser")
    font = pygame.font.Font(None, 72)

    running = True
    playing = False

    assert(os.path.isdir(log_folder))

    logpaths = get_log_names(log_folder, log_name_template)
    active_log_idx = 0
    nlogs = len(logpaths)
    logplayers = {}

    logplayers[active_log_idx] = LcmLogPlayer(
        logpaths[active_log_idx],
        args.channels
    )
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Check for keyboard events
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    playing = not playing  # Toggle play/pause state

                elif event.key == pygame.K_RIGHT:
                    logplayers[active_log_idx].reset()
                    active_log_idx = (active_log_idx + 1) % nlogs

                elif event.key == pygame.K_LEFT:
                    logplayers[active_log_idx].reset()
                    active_log_idx -= 1
                    active_log_idx = active_log_idx if active_log_idx >= 0 \
                        else nlogs - 1

        if active_log_idx not in logplayers:
            logplayers[active_log_idx] = LcmLogPlayer(
                logpaths[active_log_idx],
                args.channels
            )

        if playing:
            playing = logplayers[active_log_idx].publish_next()

        # Clear the screen
        fill_color = (0, 255, 0) if playing else (255, 0, 0)
        screen.fill(fill_color)

        # Render the current stream name
        path_stub = '/'.join(logpaths[active_log_idx].split('/')[-3:])
        text_surface = font.render(path_stub, True, (0, 0, 0))
        text_rect = text_surface.get_rect(center=(screen_width // 2, screen_height // 2))
        screen.blit(text_surface, text_rect)

        # Update the display
        pygame.display.flip()

    # Quit Pygame
    pygame.quit()


if __name__ == '__main__':
    main()
