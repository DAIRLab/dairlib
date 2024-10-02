import time

# PSG key:
# eKykJNMIaQWON7l6bWnTNQlkVMHJlqwLZBScIY6AIYktR8lUd4mHVRs3bN3RBylGcfiRIqsSIYkwx4phYp2rVDuwcP2BVRJ5RwCTIL6zMDTDcRybNxzFkWwtNTDbAF0KNIyhw3iZTtGolkjMZEWR5czxZaUmRulUcwGPxQvAedWe1UlSb8nCRFWnZqXsJJzOaAWA95uPIGjAoCxbLACwJYOkYUWs16l8RXmKlPymcE37QNiwO0i2JBCMcim7lFhObViSIKshIVkP5xhabrWBVEMgY7X7NR0QIfjOotiUQUWdN8v1c93yRIhNIYinw1iXQl2u97tecPGEFzueeISeI06xIqifIgsCIHkfNS1mcJ3yRxvsbcWWVqycSZUdQCieOAiiIO0DMtD4gqw0NFC2IEsiIhktRGhudhGxVLJxcu3GNj1wZeWpQXiUOxiwIuxUMNCa8KwhMdyB8XyIM9DOIL0qIEifwfiCR7GaFU01ZWUCVR4YckGvlsyWZyX8MsiHObicIXxUMdCh8fwBMuyr8DysMJD2IC1VIxiNwfieRDWJ1OhdahWYxJBUZhGOREywZXXeN7zHIujEoYiuYpmtpCvjYo2Gt5zddYGAFFA7ZC2x1fh1ahWLwUuXYM2q98tHIzisw0iTSKVMBkB8ZDG8R4yfZbXqNeznILjioFihMaTvY41nLnj1EUyhMgy34rytMfjVUfueMiT8cI1yIhnV0i=c084f23da2d3794045b0966d006ea1ed29fa5be625bfaa539520c3055cc5aa3fb58f77104b11171b3140671170e1c5fb81ad7eda42befe3e0a48978c1d9fd85a72714f4b2ab005b612a120184342f3aa12d1738090469ed4d296c6f616062378442542a5b1f83604c047d237a301e9103c07b2ddb8475603f83443beb266c4d86b857b83d55fa75eacc88bc477e6cd1af75d8dae04d4ca29bf98f75da7f0816443512a1e399e493d53562136a1b62a66841735a6e9fcab0990f00cc805a04f7c1b3d332167481f5f8acfa96075b0691901ba0f1862b4b851c160ad9845348b4a807938e2029ac992379a4bced540d019e98558ea9480ee781712e54eb89ac3b105857bfb5e8bc700b69be2d48b5aa98a7495b340f4871b6a18c90d8283f7165aad8528ca1d7c957bc0967462e63077ca29ec1aa79d02fe855a61117a3e7eb0550504d127629b3209522c8521b2b0e7857703925e7dd55b00f86bcd173c19e59c3687cdfc76f3693a5fbc3783ab986c9cedeb753fcb8eddaa9eb2cb7695de7cfa1cec05f37e30b7c8f92fc64b30866baff43c6fc545c27b3f9ee9a63fd8ebc8807ba6bd397c226cf2c53d4a993cea3a590c600073e0eab73d4b907620159e718a3e015bb7a7d87499484af2676c908b658ca05cbf2ffc669c9a771098b4a0e818ef461acecbe612915f1203ac322580470054b26212cba46db035ea7d4ab94da9d

import lcm
import sys
import pygame

from time import sleep
from cassie_status_lcm_backend import (
    PDPublisher,
    HeightPublisher,
    StatusSubscriber
)

from pygame_utils import TextPrint

# Initialize Pygame
pygame.init()

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
LIGHT_SKY_BLUE = (135, 206, 250)
DODGER_BLUE = (30, 144, 255)

# Set up the screen
SCREEN_WIDTH = 500
SCREEN_HEIGHT = 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Cassie Status Panel")

# Fonts
FONT = pygame.font.Font(None, 64)
BUTTON_FONT = pygame.font.Font(None, 64)

# Text input box
input_box = pygame.Rect(
    (50, SCREEN_HEIGHT // 2),
    (SCREEN_WIDTH // 3, int(BUTTON_FONT.size('test')[1] * 1.2))
)
color_inactive = LIGHT_SKY_BLUE
color_active = DODGER_BLUE
text_box_color = color_inactive
text_box_active = False
height_input_text = '0.90'
height_input_text_surface = FONT.render(
    height_input_text, True, text_box_color
)
cursor_timer = 0
cursor_visible = False

# Buttons
sh_button_size = [int(s * 1.25) for s in BUTTON_FONT.size('Set Height')]

set_height_button = pygame.Rect(
    (50, SCREEN_HEIGHT // 2 + int(input_box.size[1] * 1.1)),
    sh_button_size
)
publish_button = pygame.Rect(
    (50, SCREEN_HEIGHT // 2 + int(2.2 * input_box.size[1] * 1.1)),
    sh_button_size
)

numeric_keys = [
    pygame.K_0, pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4,
    pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9,
    pygame.K_KP0, pygame.K_KP1, pygame.K_KP2, pygame.K_KP3,
    pygame.K_KP4, pygame.K_KP5, pygame.K_KP6, pygame.K_KP7,
    pygame.K_KP8, pygame.K_KP9
]

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

pd_pub = PDPublisher(lc)
height_pub = HeightPublisher(lc)
status_sub = StatusSubscriber(lc, "INPUT_SUPERVISOR_STATUS")
status_printer = TextPrint(40, (50, 50))


# Functions
def set_height():
    h = float(height_input_text)
    height_pub.publish(h)


def publish():
    pd_pub.publish()


def get_status():
    return status_sub.status_text


def handle_keyboard(current_event, current_text):
    if text_box_active:
        if current_event.key == pygame.K_BACKSPACE:
            current_text = current_text[:-1]
        elif current_event.key in numeric_keys:
            current_text += current_event.unicode
        elif (current_event.key in [pygame.K_PERIOD, pygame.K_KP_PERIOD] and
              '.' not in current_text):
            current_text += current_event.unicode

    return current_text


def draw_buttons():
    pygame.draw.rect(screen, BLACK, set_height_button)
    pygame.draw.rect(screen, BLACK, publish_button)
    button_text1 = BUTTON_FONT.render('Set Height', True, WHITE)
    button_text2 = BUTTON_FONT.render('Publish', True, WHITE)
    screen.blit(
        button_text1, (set_height_button.x + 5, set_height_button.y + 5)
    )
    screen.blit(
        button_text2, (publish_button.x + 5, publish_button.y + 5)
    )


def handle_click(event):
    input_active = input_box.collidepoint(event.pos)
    if set_height_button.collidepoint(event.pos):
        set_height()
    if publish_button.collidepoint(event.pos):
        publish()
    return input_active


# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONUP:
            text_box_active = handle_click(event)
        if event.type == pygame.KEYDOWN:
            height_input_text = handle_keyboard(event, height_input_text)

    text_box_color = color_active if text_box_active else color_inactive
    height_input_text_surface = FONT.render(
        height_input_text, True, text_box_color
    )

    # Fill the background
    screen.fill(WHITE)

    # Render status message
    status_printer.reset()
    status_printer.print(screen, get_status(), BLACK)

    # Render input box
    pygame.draw.rect(screen, text_box_color, input_box, 2)
    screen.blit(height_input_text_surface, (input_box.x + 5, input_box.y + 5))

    # Render buttons
    draw_buttons()

    if text_box_active:
        cursor_timer += 1
        if cursor_timer >= 15:
            cursor_visible = not cursor_visible
            cursor_timer = 0
        if cursor_visible:
            cursor = FONT.render('|', True, BLACK)
            cursor_pos = height_input_text_surface.get_width() + input_box.x
            screen.blit(cursor, (cursor_pos, input_box.y))

    pygame.display.flip()
    lc.handle_timeout(5)
    time.sleep(0.02)

pygame.quit()
sys.exit()
