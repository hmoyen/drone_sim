import os
import random
import math  
import pygame
from os import listdir  
from os.path import isfile, join
from Player import Player
from Teclado import Teclado
from Menu import Menu
pygame.init()

pygame.display.set_caption("Simulador de Drone: LABDIG")

BG_COLOR = (255, 255, 255)
WIDTH, HEIGHT = 1000, 800
FPS = 60
PLAYER_VEL = 5

window = pygame.display.set_mode((WIDTH, HEIGHT))

def get_background(name):
    image = pygame.image.load(join("assets", 'Background', name))
    _, _, width, height = image.get_rect()
    tiles = []
    
    for i in range(WIDTH // width + 1):
        for j in range(HEIGHT // height + 1):
            pos = (i * width, j * height)
            tiles.append(pos)
    return tiles, image

def draw(window, background, bg_image, player):
    for tile in background:
        window.blit(bg_image, tile)
    
    player.draw(window)

    pygame.display.update()
    
def handle_move(player):
    keys = pygame.key.get_pressed()
    
    if keys[pygame.K_LEFT]:
        player.move(-player.rect.width, 0)
    if keys[pygame.K_RIGHT]:
        player.move(player.rect.width, 0)
    if keys[pygame.K_UP]:
        player.move(0, -player.rect.height)
    if keys[pygame.K_DOWN]:
        player.move(0, player.rect.height)

    

def main(windows):
    clock = pygame.time.Clock()
    background, bg_image= get_background("Blue.png")    
    
    player = Player(100, 100, 50, 50)
    listener = Teclado()
    menu = Menu(window, listener)
    state = 'menu'
    input = 'teclado' # teclado ou joystick
    
    pygame.mixer.init()
    pygame.mixer.music.load('assets/Song/BackgroundSong.mp3')
    pygame.mixer.music.play()
    
    run = True
    while run:
        clock.tick(FPS)
        
        
        if input == 'teclado':
            keys = listener.get_keys()
            
        if keys['QUIT']: #ENCERRA O JOGO
            run = False
        
        if state == 'menu':
            menu()
        else:
            player.loop(FPS)
            handle_move(player)
            draw(window, background, bg_image, player)
        
    pygame.quit()
    quit()


if __name__ == "__main__":
    main(window)        