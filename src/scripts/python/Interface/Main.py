import pygame
from os.path import join
from Player import Player
from Teclado import Teclado
from Menu import Menu

class Game:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.FPS = 60
        self.BG_COLOR = (255, 255, 255)
        self.PLAYER_VEL = 5
        self.window = pygame.display.set_mode((width, height))

        pygame.display.set_caption("Simulador de Drone: LABDIG")
        pygame.init()

    def get_background(self, name):
        image = pygame.image.load(join("assets", 'Background', name))
        _, _, width, height = image.get_rect()
        tiles = []
        
        for i in range(self.width // width + 1):
            for j in range(self.height // height + 1):
                pos = (i * width, j * height)
                tiles.append(pos)
        return tiles, image

    def draw(self, background, bg_image, player):
        for tile in background:
            self.window.blit(bg_image, tile)
        
        player.draw(self.window)

        pygame.display.update()
        
    def handle_move(self, player):
        keys = pygame.key.get_pressed()
        
        if keys[pygame.K_LEFT]:
            player.move(-player.rect.width, 0)
        if keys[pygame.K_RIGHT]:
            player.move(player.rect.width, 0)
        if keys[pygame.K_UP]:
            player.move(0, -player.rect.height)
        if keys[pygame.K_DOWN]:
            player.move(0, player.rect.height)

    def run(self):
        clock = pygame.time.Clock()
        background, bg_image = self.get_background("Blue.png")    
        
        player = Player(100, 100, 50, 50)
        listener = Teclado()
        menu = Menu(self.window, listener)
        state = 'menu'
        input_device = 'teclado' # teclado ou joystick
        
        pygame.mixer.init()
        pygame.mixer.music.load('assets/Song/BackgroundSong.mp3')
        pygame.mixer.music.play()
        
        run = True
        while run:
            clock.tick(self.FPS)
            
            
            if input_device == 'teclado':
                keys = listener.get_keys()
                
            if keys['QUIT']: #ENCERRA O JOGO
                run = False
            
            
            if state == 'menu':
                menu()
            else:
                player.loop(self.FPS)
                self.handle_move(player)
                self.draw(background, bg_image, player)
            
        pygame.quit()
        quit()

# Example of how to use the class:
if __name__ == "__main__":
    game = Game(1000, 800)
    game.run()
