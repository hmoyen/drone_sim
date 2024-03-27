import pygame
class Player(pygame.sprite.Sprite):
    COLOR = (255, 0, 0)
    def __init__(self, x, y, width, height):
        self.rect = pygame.Rect(x, y, width, height)
        
    def move(self, dx, dy):
        self.rect.x += dx
        self.rect.y += dy
        
    def loop(self, fps):
        pass

    def draw(self, win):
        pygame.draw.rect(win, self.COLOR, self.rect)