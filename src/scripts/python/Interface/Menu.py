from typing import Any
import pygame
from Teclado import Teclado
import os


class Menu():
    TIPO = "MODO" #MODO, VIDA ou MAPA
    MODOS = ['FACIL', 'MEDIO', 'DIFICIL'] #FACIL, MEDIO ou DIFICIL
    MODO = 0 #0 a 2
    VIDAS = 1 #1 a 5
    MAPA = 0 #0, 1, 2, 3 ou 4
    
    def __init__(self, window, listener):
        self.window = window
        self.listener = listener
        
    def modo(self):
        selected_mode = self.MODOS[self.MODO]

        font_path = os.path.join("/home/lena/drone_sim/src/scripts/python/Interface/assets", "Fonts", "ARCADECLASSIC.ttf")
        font = pygame.font.Font(font_path, 36)

        text = font.render("VOCE SELECIONOU O MODO {}".format(selected_mode), True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (self.window.get_width() // 2, self.window.get_height() // 2)
        self.window.fill((255, 255, 255))
        self.window.blit(text, text_rect)
        pygame.display.update()
        

        # DEVOLVE UM DICIONARIO COM OS COMANDOS ACIONADOS (True ou False)
        # ENTER confirma o modo
        if self.listener.teclas['ENTER']:
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Confirm.mp3')
            sound_effect.play()
            self.TIPO = "VIDA"
        # SE NÃO, ALTERA O MODO
        if self.listener.teclas['UP']:
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Up.mp3')
            sound_effect.play()
            self.MODO = (self.MODO + 1) % 3
        if self.listener.teclas['DOWN']:
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Up.mp3')
            sound_effect.play()
            if self.MODO == 0:
                self.MODO = 2
            else:
                self.MODO = (self.MODO - 1) % 3
        
    def vida(self):
        selected_mode = self.MODOS[self.MODO]
        selected_lives = self.VIDAS

        font_path = os.path.join("/home/lena/drone_sim/src/scripts/python/Interface/assets", "Fonts", "ARCADECLASSIC.ttf")
        font = pygame.font.Font(font_path, 36)

        text = font.render("VOCE SELECIONOU O MODO {} COM {} VIDAS".format(selected_mode, selected_lives), True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (self.window.get_width() // 2, self.window.get_height() // 2)
        self.window.fill((255, 255, 255))
        self.window.blit(text, text_rect)
        pygame.display.update()
        
        # DEVOLVE UM DICIONARIO COM OS COMANDOS ACIONADOS (True ou False)
        # ENTER confirma o modo
        if self.listener.teclas['ENTER']:
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Confirm.mp3')
            sound_effect.play()
            self.TIPO = "MAPA"
        # SE NÃO, ALTERA O MODO
        if self.listener.teclas['UP']:
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Up.mp3')
            sound_effect.play()
            if self.VIDAS == 5:
                self.VIDAS = 1
            else:
                self.VIDAS = (self.VIDAS + 1) % 6
        if self.listener.teclas['DOWN']:
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Up.mp3')
            sound_effect.play()
            if self.VIDAS == 1:
                self.VIDAS = 5
            else:
                self.VIDAS = (self.VIDAS - 1) % 6
    
    def mapa(self):
        selected_mode = self.MODOS[self.MODO]
        selected_lives = self.VIDAS
        selected_map = self.MAPA

        font_path = os.path.join("/home/lena/drone_sim/src/scripts/python/Interface/assets", "Fonts", "ARCADECLASSIC.ttf")
        font = pygame.font.Font(font_path, 36)

        text = font.render("VOCE SELECIONOU O MODO {} COM {} VIDAS NO MAPA {}".format(selected_mode, selected_lives, selected_map), True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (self.window.get_width() // 2, self.window.get_height() // 2)
        self.window.fill((255, 255, 255))
        self.window.blit(text, text_rect)
        pygame.display.update()
        
        # DEVOLVE UM DICIONARIO COM OS COMANDOS ACIONADOS (True ou False)
        # ENTER confirma o modo
        if self.listener.teclas['ENTER']:
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Confirm.mp3')
            sound_effect.play()
            self.TIPO = "AGUARDA"
        # SE NÃO, ALTERA O MODO
        if self.listener.teclas['UP']:
            self.MAPA = (self.MAPA + 1) % 5
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Up.mp3')
            sound_effect.play()
        if self.listener.teclas['DOWN']:
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Up.mp3')
            sound_effect.play()
            if self.MAPA == 0:
                self.MAPA = 4
            else:
                self.MAPA = (self.MAPA - 1) % 5
    
    def aguarda(self):
        selected_mode = self.MODOS[self.MODO]
        selected_lives = self.VIDAS
        selected_map = self.MAPA

        font_path = os.path.join("/home/lena/drone_sim/src/scripts/python/Interface/assets", "Fonts", "ARCADECLASSIC.ttf")
        font = pygame.font.Font(font_path, 36)
        text_out = 'VOCE SELECIONOU O MODO {} COM {} VIDAS NO MAPA {}\nPRESSIONE ENTER PARA INICIAR'.format(selected_mode, selected_lives, selected_map)
        self.window.fill((255, 255, 255))
        self.blit_text(surface = self.window, text = text_out, pos = (10, 10),font = font)
        pygame.display.update()
        if self.listener.teclas['ENTER']:
            sound_effect = pygame.mixer.Sound('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/Confirm.mp3')
            sound_effect.play()
            self.TIPO = "MODO"
            
    def blit_text(self, surface, text, pos, font, color=pygame.Color('black')):
        words = [word.split(' ') for word in text.splitlines()]  # 2D array where each row is a list of words.
        space = font.size(' ')[0]  # The width of a space.
        max_width, max_height = surface.get_size()
        x, y = pos
        for line in words:
            line_width = sum(font.size(word)[0] for word in line) + space * (len(line) - 1)
            x = (max_width - line_width) // 2  # Calculate the x position to center the line.
            y = (max_height - sum(font.size(word)[1] for word in line)) // 2  # Calculate the y position to center the text vertically.
            for word in line:
                word_surface = font.render(word, 0, color)
                word_width, word_height = word_surface.get_size()
                surface.blit(word_surface, (x, y))
                x += word_width + space
            y += word_height  # Start on new row.
                    
    def __call__(self):
        if self.TIPO == "MODO":
            self.modo()
        elif self.TIPO == "VIDA":
            self.vida()
        elif self.TIPO == "MAPA":
            self.mapa()
        elif self.TIPO == "AGUARDA":
            self.aguarda()
