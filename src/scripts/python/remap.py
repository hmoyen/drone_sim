#!/usr/bin/env python
import subprocess
import rospy
import time
from mav import MAV2
from std_msgs.msg import String
from std_msgs.msg import Int32
from gazebo_msgs.msg import ContactsState

def input_callback(data):
    global collision_detected, output_message

    if data.states:
        # Collision detected
        # print("DETECTED")
        output_message = 1
        collision_detected = True
    else:
        # No collision
        output_message = 0 
        collision_detected = False

def velocity_callback(data):
    global vel_message
    print(data.data)

    if data.data == 1:
        vel_message = 1
    else:
        vel_message = 0


if __name__ == '__main__':
    # Initialize the ROS node
    subprocess.Popen(["xterm", "-e", "bash", "-c", "sim_vehicle.py -v ArduCopter -f gazebo-iris"])
    subprocess.Popen(["xterm", "-e", "bash", "-c", "roslaunch sky_sim sky_sim.launch"])
    
    rospy.init_node('transforming_node', anonymous=True)

    # Initialize the output publisher
    output_pub = rospy.Publisher('/broker/collision', Int32, queue_size=10)
    rospy.Subscriber('/drone_bumper', ContactsState, input_callback, queue_size=10)
    rospy.Subscriber('/ros/set_vel', Int32, velocity_callback, queue_size = 10)
    global collision_detected, output_message, dr
    collision_detected = False
    output_message = 0
    vel_message= 0

    rate = rospy.Rate(10)  # 10Hz
    dr = MAV2()

    while not dr.drone_state.armed:
        try:
            dr.takeoff(10)
            print("Takeoff sent")
            rospy.sleep(5)
        except KeyboardInterrupt:
            print("Interrumpted")

    while not rospy.is_shutdown():
        output_pub.publish(output_message)
        # print(vel_message)
        dr.set_vel(vel_message,0,0,0)
        rate.sleep()
