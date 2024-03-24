import rospy
import time
import random
import paho.mqtt.client as mqtt
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool
from mav import MAV2  # Assuming MAV2 class is defined in the mav.py file

class Drone:
    def __init__(self):
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

        # MQTT client initialization
        self.mqtt_client = MQTTClient()

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback, queue_size=10)
        rospy.Subscriber('/drone_bumper', ContactsState, self.bumper_callback, queue_size=10)

    def bumper_callback(self, msg):
        current_time = time.time()
        
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
                self.collision_buffer.append(1)  # Adiciona mensagem de "com colisão" ao buffer
                self.last_publish_time = current_time

                if len(self.collision_buffer) >= 1:
                    self.mqtt_client.publish_collision(self.collision_buffer[0])
                    self.collision_buffer.clear()
                
        else:
            # No collision
            self.collision_detected = False
            self.no_collision_count += 1
            if self.no_collision_count >= self.MAX_NO_COL_COUNT:
                self.publish_no_collision()

    def publish_no_collision(self):
        if time.time() - self.last_publish_time >= self.publish_interval:
            self.mqtt_client.publish_collision(0)  # Publica 5 mensagens de "sem colisão"
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
        self.client.on_message = self.on_message

        self.Broker = "192.168.0.27"
        self.Port = 1883
        self.KeepAlive = 600
        self.TopicCollision = "drone_collision"

        self.client.connect(self.Broker, self.Port, self.KeepAlive)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT Broker with result code " + str(rc))

    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.payload))

    def publish_collision(self, collision):
        print("Published:", collision)
        self.client.publish(self.TopicCollision, payload=collision, qos=0, retain=False)

def main():
    drone = Drone()
    rospy.spin()

if __name__ == '__main__':
    main()
