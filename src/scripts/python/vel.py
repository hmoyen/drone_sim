import subprocess
import time
import random
import paho.mqtt.client as mqtt
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ContactsState
from mav import MAV2  # Assuming MAV2 class is defined in the mav.py file

class Drone:
    def __init__(self):
         # MQTT client initialization
        # self.mqtt_client.le_terminal()
        # self.mqtt_client = MQTTClient()
        rospy.init_node('mqtt_to_ros_bridge')
        self.dr = MAV2()
        self.started = False
        self.collision_detected = False
        self.last_collision_time = 0
        self.last_publish_time = 0
        self.publish_interval = 0.1  # Intervalo de publicação de 10Hz
        self.MAX_NO_COL_COUNT = 15  # Número máximo de mensagens "sem colisão"
        self.no_collision_count = 0
        self.collision_buffer = []

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback, queue_size=10)
        rospy.Subscriber('/drone_bumper', ContactsState, self.bumper_callback, queue_size=10)

        
        try:
            self.dr.takeoff(10)
            print("Takeoff sent")
            rospy.sleep(5)
        except KeyboardInterrupt:
                print("Interrumpted")
        

    def bumper_callback(self, msg):
        current_time = time.time()
        
        # self.dr.set_vel(self.mqtt_client.velocity)
        
        if not self.started:
            self.collision_detected = False
            self.no_collision_count = 0
            self.publish_no_collision()

        elif msg.states:
            # Collision detected
            self.collision_detected = True
            self.last_collision_time = current_time
            self.no_collision_count = 0
            
            if current_time - self.last_publish_time >= self.publish_interval:
                # self.collision_buffer.append(1) 
                self.last_publish_time = current_time
                client.publish_collision(self.collision_buffer[0])

                if len(self.collision_buffer) >= 5:
                    client.publish_collision(self.collision_buffer[0])
                    self.collision_buffer.clear()
                
        else:
            # No collision
            self.collision_detected = False
            self.no_collision_count += 1
            if self.no_collision_count >= self.MAX_NO_COL_COUNT:
                self.publish_no_collision()

    def publish_no_collision(self):
        if time.time() - self.last_publish_time >= self.publish_interval:
            client.publish_collision(0)  # Publica 5 mensagens de "sem colisão"
            self.last_publish_time = time.time()

    def pose_callback(self, msg):
        if msg.pose.position.z < 3:
            self.started = False
        else:
            self.started = True

class MQTTClient:
    def __init__(self):
        self.client = mqtt.Client(client_id=f'python-mqtt-{random.randint(0, 1000)}')
        self.client.on_connect = self.on_connect
        # self.drone = Drone()
        # self.client.on_message = self.on_message

        self.Broker = "172.20.10.2"
        self.Port = 1883
        self.KeepAlive = 600
        self.TopicCollision = "/broker/drone_collision"
        self.TopicWorld = "world"
        self.TopicVelocity = "/broker/set_vel"
        self.TopicStart = "init"
        self.TopicEnd = "end"
        self.velocity = [0,0,0,0]

        self.begun = False
        self.end = True

        self.client.connect(self.Broker, self.Port, self.KeepAlive)
        # self.client.loop_forever()
        self.client.subscribe("/broker/set_vel")
        self.client.message_callback_add("/broker/set_vel", self.set_vel_callback)
        # self.client.message_callback_add("$SYS/broker/messages/#", self.on_message_msgs)
        # self.client.message_callback_add("$SYS/broker/bytes/#", self.on_message_bytes)
        # self.client.on_message = self.on_message
        # self.client.connect("mqtt.eclipseprojects.io", 1883, 60)
        # self.client.subscribe("$SYS/#", 0)
        self.client.loop_forever()
    
    def initialize_sub(self):
        self.client.subscribe("/broker/")
        self.client.message_callback_add("/broker/set_vel", self.set_vel_callback)

        # self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT Broker with result code " + str(rc))

    def set_vel_callback(self, client, userdata, msg):

        print("Received")
        self.velocity = msg.payload.decode()
        print(self.velocity)
        return self.velocity

    def get_vel(self):
        return self.velocity

    # def on_message(self, client, userdata, msg):

    #     print(msg.payload.decode())

    #     if msg.topic == self.TopicStart:

    #         self.begun = int(msg.payload.decode())
            
    #         if self.begun == True:
    #             interface.restart()
        
    #     if msg.topic == self.TopicEnd:

    #         self.end = int(msg.payload.decode())

    #         if self.end == True:
    #             interface.end()
                

    #     if msg.topic == self.TopicWorld:
            
    #         self.client.newmsg = True
    #         # Handle the message from "world" topic
            
    #         self.launch_choice = int(msg.payload.decode())
    #         print(self.launch_choice)
    #         interface.run_command_in_terminal(self.launch_choice)
        
    #     else:
    #         print("Game is initializing...")

    def le_terminal(self, verbose=1):
        self.client.newmsg = False

        # Fica em loop infinito ate receber uma nova mensagem
        while not self.client.newmsg:
            self.client.loop_start()
            print("Message")
            time.sleep(2)
            self.client.loop_stop()

        return self.launch_choice
    
    # def on_message_msgs(self,mosq, obj, msg):
    # # This callback will only be called for messages with topics that match
    # # $SYS/broker/messages/#
    #     print("MESSAGES: " + msg.topic + " " + str(msg.qos) + " " + str(msg.payload))


    # def on_message_bytes(self,mosq, obj, msg):
    #     # This callback will only be called for messages with topics that match
    #     # $SYS/broker/bytes/#
    #     print("BYTES: " + msg.topic + " " + str(msg.qos) + " " + str(msg.payload))


    # def on_message(self, mosq, obj, msg):
    #     # This callback will be called for messages that we receive that do not
    #     # match any patterns defined in topic specific callbacks, i.e. in this case
    #     # those messages that do not have topics $SYS/broker/messages/# nor
    #     # $SYS/broker/bytes/#
    #     print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

    def publish_collision(self, collision):
        # print("Published:", collision)
        self.client.publish(self.TopicCollision, payload=collision, qos=0, retain=False)

class Interface:

    def __init__(self):

        # self.restart()
        pass

    def run_command_in_terminal(self, choice):
        # Retrieve the chosen launch command
        chosen_launch = list(self.launches.values())[choice - 1]

        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "roslaunch sky_sim sky_sim.launch"])
    
    def restart(self):

        subprocess.Popen(["xterm", "-e", "bash", "-c", "sim_vehicle.py -v ArduCopter -f gazebo-iris"])
        
        self.launches = {
            "Launch 1": "roslaunch sky_sim sky_sim.launch",
            "Launch 2": "roslaunch sky_sim runaway.launch",
            "Launch 3": "roslaunch sky_sim indoor.launch"
        }

        # Prompt user to choose a launch
        print("Available Launches:")
        for index, launch_name in enumerate(self.launches.keys(), 1):
            print(f"{index}. {launch_name}")
    
    def end(self):

        print("Closing terminals...")


def main():
    global interface  # Make interface accessible in on_message method
    global drone
    global client
    
    # interface = Interface()
    subprocess.Popen(["xterm", "-e", "bash", "-c", "sim_vehicle.py -v ArduCopter -f gazebo-iris"])
    subprocess.Popen(["xterm", "-e", "bash", "-c", "roslaunch sky_sim sky_sim.launch"])
    
    drone = Drone()
    client = MQTTClient()
    rospy.spin()

    # drone.mqtt_client.initialize_sub()



if __name__ == '__main__':
    main()
