import subprocess
import rospy
import time
import random
from mav import MAV2
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SpawnModel, DeleteModel
import math
import os
import random
import math  
import pygame
import signal
from os import listdir  
from os.path import isfile, join
from Player import Player
from Teclado import Teclado
from Main import Game
from Menu import Menu
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


# Define the size of each grid cell
cell_size = 10

prev_row = None
prev_col = None
spawned_model_names=[]

global collision_detected, output_message, dr, started, last_collision_time, end

# Define a list of model paths corresponding to different objects
model_paths = {
    # 0: "/home/lena/sky_ws/src/sky_sim/models/palm_tree/model.sdf",
    0: "/home/lena/sky_ws/src/sky_sim/models/salon/model.sdf",
    1: "/home/lena/sky_ws/src/sky_sim/models/thrift_shop/model.sdf",
    2: "/home/lena/sky_ws/src/sky_sim/models/law_office/model.sdf"
    # 2: "/home/lena/sky_ws/src/sky_sim/models/tower_crane/model.sdf"
    # Add more model paths as needed
}

# Define the 16x4 matrix
matrix_1 = [
    [0, 0, 0, 0],
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, 0, 0, 1],
    [0, 0, 1, 0],
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [0, 1, 0, 0],
    [1, 0, 0, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1],
    [0, 1, 0, 0]
]

matrix_2 = [
    [0, 0, 0, 0],
    [1, 0, 1, 0],
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [0, 1, 0, 1],
    [0, 0, 0, 1],
    [1, 0, 1, 0],
    [1, 0, 0, 1],
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 0, 1],
    [1, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [0, 1, 0, 1]
]

matrixes = [matrix_1,matrix_2]

def get_square_coordinates(x, y, cell_size):
    """
    Function to calculate the grid coordinates (row and column) of the square based on x and y positions.
    """
    row = int(y / cell_size)  # Calculate row
    col = int(x / cell_size)  # Calculate column
    return row, col

def publish_movement_flags():
    """
    Function to publish movement flags for each direction.
    """
    right_pub.publish(1 if moved_right else 0)
    left_pub.publish(1 if moved_left else 0)
    forward_pub.publish(1 if moved_forward else 0)
    backward_pub.publish(1 if moved_backward else 0)

def monitor_movement(row, col):
    """
    Function to monitor movement and set movement flags.
    """
    global prev_row, prev_col, moved_right, moved_left, moved_forward, moved_backward

    # Reset movement flags
    moved_right = False
    moved_left = False
    moved_forward = False
    moved_backward = False

    if prev_row is not None and prev_col is not None:
        # Compare current position with previous position to detect movement direction
        if row > prev_row:
            moved_right = True
            print("Right")
        elif row < prev_row:
            moved_left = True
            print("Left")
        if col > prev_col:
            moved_backward = True
            print("Back")
        elif col < prev_col:
            moved_forward = True
            print("Forward")

    # Update previous position
    prev_row = row
    prev_col = col

def pose_callback(data):
    """
    Callback function to handle object pose updates.
    """
    global started

    if data.pose.position.z < 2:
        started = False
    else:
        started = True

    # Get x and y positions from object's pose
    x = data.pose.position.x
    y = data.pose.position.y

    # Calculate grid coordinates of the square
    row, col = get_square_coordinates(x, y, cell_size)

    # Monitor movement relative to the matrix
    monitor_movement(row, col)

    # Publish movement flags
    publish_movement_flags()

def spawn_model(model_path, model_name, model_pose, orientation=None):
    """
    Function to spawn a model in Gazebo.
    """
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        with open(model_path, "r") as f:
            model_xml = f.read()
        if orientation is not None:
            model_pose.orientation = orientation.orientation
        spawn_sdf(model_name, model_xml, "/", model_pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr("Spawn model service call failed: {0}".format(e))

def delete_model(model_name):
    """
    Function to delete a model from Gazebo.
    """
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model(model_name)
    except rospy.ServiceException as e:
        rospy.logerr("Delete model service call failed: {0}".format(e))

def input_callback(data):
    global collision_detected, output_message, last_collision_time

    if data.states:
        # Collision detected
        last_collision_time = rospy.get_rostime()  # Update last collision time

    # Calculate the time since the last collision
    time_since_collision = rospy.get_rostime() - last_collision_time

    if time_since_collision <= rospy.Duration(1):
        # Publish a collision message with value 1
        output_message = 1
    else:
        # Publish a collision message with value 0
        output_message = 0

def velocity_x_callback(data):
    global vel_x_message
    # print(data.data)

    if data.data == 1:
        vel_x_message = 5
    elif data.data == -1:
        vel_x_message = -5
    else:
        vel_x_message = 0

def velocity_y_callback(data):
    global vel_y_message
    # print(data.data)

    if data.data == 1:
        vel_y_message = 5
    elif data.data == -1:
        vel_y_message = -5
    else:
        vel_y_message = 0

def world_callback(data):

    global world

    if not end:
        world = int(data.data)
        construct_world(matrixes[world])


def init_callback(data):

    global init

    if data.data == 1:
        init = True
    else:
        init = False

def construct_world(matrix):
        
        global spawned_model_names
        # Define the path to the box model SDF file
        box_model_path = "/home/lena/sky_ws/src/sky_sim/models/grey_wall/model.sdf"

        # List to store the names of spawned models
        spawned_model_names = []

        # Define the dimensions of the box and the gap between the matrix and the wall of boxes
        box_dimensions = [cell_size, cell_size, 20]  # Assuming the wall is 2 meters tall
        gap = 1  # Gap between the matrix and the wall

        # Iterate through the outer boundary of the matrix to spawn boxes
        for i in range(len(matrix) + 2):
            for j in range(len(matrix[0]) + 2):
                # If we're at a position outside the matrix, spawn a box
                if i == 0 or i == len(matrix) + 1 or j == 0 or j == len(matrix[0]) + 1:
                    # Calculate the position of the box
                    x = (j - 1) * cell_size - gap * (j == 0 or j == len(matrix[0]) + 1)
                    y = (i - 1) * cell_size - gap * (i == 0 or i == len(matrix) + 1)
                    z = box_dimensions[2] / 2  # Center the box on the ground

                    # Calculate the orientation of the box (rotation around z-axis)
                    orientation = Pose()
                    if i == 0 or i == len(matrix) + 1:  # Along the y-axis
                        orientation.orientation.w = 1
                    else:  # Along the x-axis
                        orientation.orientation.z = math.sin(math.pi / 4)
                        orientation.orientation.w = math.cos(math.pi / 4)

                    # Define the pose of the box model
                    model_pose = Pose()
                    model_pose.position.x = x
                    model_pose.position.y = y
                    model_pose.position.z = z

                    # Spawn the box model in Gazebo
                    model_name = "box_" + str(i) + "_" + str(j)  # Unique model name
                    spawn_model(box_model_path, model_name, model_pose, orientation)
                    # Append the spawned model name to the list
                    spawned_model_names.append(model_name)

        # Iterate through each cell in the matrix to randomly spawn objects
        for i in range(len(matrix)):
            for j in range(len(matrix[0])):
                # Check if the cell is not empty
                if matrix[i][j] != 0:
                    # Randomly select a model path based on the matrix value
                    model_path = random.choice(model_paths)

                    # Calculate the position of the object
                    x = j * cell_size
                    y = i * cell_size
                    z = 0  # Assuming the objects are placed on the ground

                    # Define the pose of the model
                    model_pose = Pose()
                    model_pose.position.x = x
                    model_pose.position.y = y
                    model_pose.position.z = z

                    # Spawn the model in Gazebo
                    model_name = "object_" + str(i) + "_" + str(j)  # Unique model name
                    spawn_model(model_path, model_name, model_pose)
                    # Append the spawned model name to the list
                    spawned_model_names.append(model_name)

def destruct_world(spawned_model_names):

    # Remove the spawned models
    for model_name in spawned_model_names:
        delete_model(model_name)

def respawn_model(model_name, pose):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = pose[0]
        model_state.pose.position.y = pose[1]
        model_state.pose.position.z = pose[2]
        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.linear.z = 0
        model_state.twist.angular.x = 0
        model_state.twist.angular.y = 0
        model_state.twist.angular.z = 0
        model_state.reference_frame = 'world'
        set_model_state(model_state)
        rospy.loginfo("Model {} respawned at pose {}".format(model_name, pose))
    except rospy.ServiceException as e:
        rospy.logerr("Failed to respawn model: {}".format(e))

def init_gazebo():

    subprocess.Popen(["xterm", "-e", "bash", "-c", "roslaunch sky_sim sky_sim.launch"])

def end_gazebo():
    # Find and kill the Gazebo process
    try:
        # Find and kill the Gazebo process
        ps_output_gazebo = subprocess.check_output(["pgrep", "-f", "gzserver"])
        gazebo_pid = int(ps_output_gazebo.decode().strip())
        os.kill(gazebo_pid, signal.SIGTERM)
        print("Gazebo process terminated successfully.")

    except subprocess.CalledProcessError:
        print("Error: Process not found.")
    except Exception as e:
        print("Error:", e)

def end_ardupilot():
        # Find and kill the sim_vehicle.py process
        try:
            ps_output_sim_vehicle = subprocess.check_output(["pgrep", "-f", "sim_vehicle.py -v ArduCopter -f gazebo-iris"])
            sim_vehicle_pid = int(ps_output_sim_vehicle.decode().strip())
            os.kill(sim_vehicle_pid, signal.SIGTERM)
            print("sim_vehicle.py process terminated successfully.")

        except subprocess.CalledProcessError:
            print("Error: Process not found.")
        except Exception as e:
            print("Error:", e)

def start_ardupilot():

        # Initialize the ROS node
    subprocess.Popen(["xterm", "-e", "bash", "-c", "sim_vehicle.py -v ArduCopter -f gazebo-iris"])

def end_callback(data):

    global end
    if data.data == 1:
         end = True
    else:
        end = False

if __name__ == '__main__':

    global init, world

    if rospy.is_shutdown:
        subprocess.Popen(["xterm", "-e", "bash", "-c", "roscore"])

    rospy.init_node('transforming_node', anonymous=True)

    rospy.Subscriber('/ros/world', Int32, world_callback, queue_size=10)
    rospy.Subscriber('/ros/init', Int32, init_callback, queue_size=10)
    rospy.Subscriber('/ros/end', Int32, end_callback, queue_size=10)
    # Initialize the output publisher
    output_pub = rospy.Publisher('/broker/collision', Int32, queue_size=10)
    rospy.Subscriber('/drone_bumper', ContactsState, input_callback, queue_size=10)
    rospy.Subscriber('/ros/set_vel_x', Int32, velocity_x_callback, queue_size=10)
    rospy.Subscriber('/ros/set_vel_y', Int32, velocity_y_callback, queue_size=10)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback, queue_size=10)
    # Initialize publishers for movement flags
    right_pub = rospy.Publisher('/movement/right', Int32, queue_size=10)
    left_pub = rospy.Publisher('/movement/left', Int32, queue_size=10)
    forward_pub = rospy.Publisher('/movement/forward', Int32, queue_size=10)
    backward_pub = rospy.Publisher('/movement/backward', Int32, queue_size=10)

    init = False
    # game = Game(1000, 800)

    # while not init:
    #     game.run()
    collision_detected = False
    output_message = 0
    vel_y_message = 0
    vel_x_message = 0
    started = False
    last_collision_time = rospy.Time.now()
    
    reset = False
    end = False
    start_ardupilot()
    init_gazebo()
    rate = rospy.Rate(5)  # 10Hz
    dr = MAV2()

    # init_gazebo()

    while True:


        # subprocess.Popen(["xterm", "-e", "bash", "-c", "roslaunch mqtt_client standalone.launch"])

        collision_detected = False
        output_message = 0
        vel_y_message = 0
        vel_x_message = 0
        started = False
        last_collision_time = rospy.Time.now()

        while not started:
            try:
                dr.takeoff(10)
                print("Takeoff sent")
                rospy.sleep(5)
            except KeyboardInterrupt:
                print("Interrupted")

        while not end:
            output_pub.publish(output_message)
            dr.set_vel(vel_x_message, vel_y_message, 0, 0)
            rate.sleep()
            # if reset:
            #     end_gazebo()
            #     init_gazebo()
            #     break
        
        destruct_world(spawned_model_names)
        respawn_model('iris',[0,0,0])
        print("spawned")
        # end_ardupilot()
        # end_gazebo()
        end = False
        started = False
        # end_gazebo()
        # break

        init = False