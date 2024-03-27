import pygame

class Teclado():
    def __init__(self):
        self.teclas = {'UP': False, 'DOWN': False, 'LEFT': False,
                       'RIGHT': False, 'ENTER': False, 'QUIT': False}
    
    def get_keys(self):
        
        for k in self.teclas.keys():
                self.teclas[k] = False 
                
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.teclas['QUIT'] = True
                
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.teclas['UP'] = True
                    print('UP pressed')
                if event.key == pygame.K_DOWN:
                    self.teclas['DOWN'] = True
                    print('DOWN pressed')
                if event.key == pygame.K_LEFT:
                    self.teclas['LEFT'] = True
                    print('LEFT pressed')
                if event.key == pygame.K_RIGHT:
                    self.teclas['RIGHT'] = True
                    print('RIGHT pressed')
                if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                    self.teclas['ENTER'] = True
                    print('ENTER pressed')
                    
        return self.teclas