import pygame


class TextPrint:
    def __init__(self, font_size, upper_left):
        self.upper_left = upper_left
        self.x, self.y = upper_left
        self.font = pygame.font.Font(None, font_size)
        self.line_height = int(1.1 * self.font.size('TEST')[-1])
        self.new_line_height = int(self.line_height * 1.25)

    def print(self, screen, text_string, color):
        text_list = text_string.split("\n")
        for string in text_list:
            text_bitmap = self.font.render(string, True, color)
            screen.blit(text_bitmap, [self.x, self.y])
            self.y += self.line_height
        self.y += self.new_line_height

    def reset(self):
        self.x, self.y = self.upper_left

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10
