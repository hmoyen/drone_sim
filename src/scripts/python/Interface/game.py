import os
import subprocess
import rospy
import time
import pygame
from std_msgs.msg import Int32
from gazebo_msgs.msg import ContactsState
from Player import Player
from Teclado import Teclado
from Menu import Menu
from os import listdir  
from os.path import isfile, join
from Player import Player
from Teclado import Teclado
from Menu import Menu
from mav import MAV2


# Initialize Pygame
pygame.init()

# Define Pygame window parameters
pygame.display.set_caption("Simulador de Drone: LABDIG")
BG_COLOR = (255, 255, 255)
WIDTH, HEIGHT = 1000, 800
FPS = 60
PLAYER_VEL = 5
window = pygame.display.set_mode((WIDTH, HEIGHT))

# ROS Callbacks
def input_callback(data):
    global collision_detected, output_message

    if data.states:
        # Collision detected
        output_message = 1
        collision_detected = True
    else:
        # No collision
        output_message = 0 
        collision_detected = False

def velocity_callback(data):
    global vel_message
    if data.data == 1:
        vel_message = 1
    else:
        vel_message = 0

def get_background(name):
    image = pygame.image.load(join("/home/lena/drone_sim/src/scripts/python/Interface/assets/", 'Background', name))
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

    
# ROS Initialization
def init_ros():
    # Launch ROS nodes
    subprocess.Popen(["xterm", "-e", "bash", "-c", "sim_vehicle.py -v ArduCopter -f gazebo-iris"])
    subprocess.Popen(["xterm", "-e", "bash", "-c", "roslaunch sky_sim sky_sim.launch"])
    
    # Initialize ROS node
    rospy.init_node('transforming_node', anonymous=True)
    global dr
    dr = MAV2()
    # Initialize ROS subscribers and publishers
    global output_pub, collision_detected, output_message, vel_message
    output_pub = rospy.Publisher('/broker/collision', Int32, queue_size=10)
    rospy.Subscriber('/drone_bumper', ContactsState, input_callback, queue_size=10)
    rospy.Subscriber('/ros/set_vel', Int32, velocity_callback, queue_size=10)
    collision_detected = False
    output_message = 0
    vel_message = 0

# Pygame game loop
def game_loop():
    clock = pygame.time.Clock()
    background, bg_image = get_background("Blue.png")    
    player = Player(100, 100, 50, 50)
    listener = Teclado()
    menu = Menu(window, listener)
    state = 'menu'
    input = 'teclado' # teclado ou joystick
    
    pygame.mixer.init()
    pygame.mixer.music.load('/home/lena/drone_sim/src/scripts/python/Interface/assets/Song/BackgroundSong.mp3')
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

# ROS publishing and control loop
def ros_loop():
    rate = rospy.Rate(10)  # 10Hz

    while not dr.drone_state.armed:
        try:
            dr.takeoff(10)
            print("Takeoff sent")
            rospy.sleep(5)
        except KeyboardInterrupt:
            print("Interrumpted")

    while not rospy.is_shutdown():
        output_pub.publish(output_message)
        dr.set_vel(vel_message, 0, 0, 0)
        rate.sleep()

# Entry point of the program
if __name__ == "__main__":
    init_ros()  # Initialize ROS
    game_loop()  # Start Pygame game loop
    ros_loop()  # Start ROS control loop
