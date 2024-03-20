import paho.mqtt.client as mqtt
import rospy
from mav import MAV2  # Seu arquivo MAV2.py com a classe MAV2
import time

# Defina a função de callback para o recebimento de mensagens MQTT
def on_message(client, userdata, msg):
    # Decodifica a mensagem recebida do tópico MQTT "out"
    client.newmsg = True    
    client.msg = msg.payload.decode("utf-8")

def le_terminal(topico='out', verbose=1):
    client.newmsg = False

    # Fica em loop infinito ate receber uma nova mensagem
    while not client.newmsg:
        client.loop_start()
        print("Message")
        time.sleep(2)
        client.loop_stop()

    if verbose == 1:
        print("> " + client.msg)
        try:
            dr.go_to_local([3, 0, 0])  # Isso publicará um comando para mover 1 metro no eixo X
            time.sleep(5)
        except KeyboardInterrupt:
            print("foi")

    return client.msg


rospy.init_node('mqtt_to_ros_bridge')
dr = MAV2()
client = mqtt.Client()
client.on_message = on_message

# Conecte-se ao broker MQTT
client.connect("192.168.0.2", 1883, 60)

# Inscreva-se no tópico MQTT "out"
client.subscribe('out')

try:
    dr.takeoff(5)
    print("Takeoff sent")
except KeyboardInterrupt:
        print("foi")
rospy.sleep(5)
rospy.spin()
le_terminal()
# # Inicie o loop de recebimento de mensagens MQTT
# client.loop_start()

# # Desconecte-se do broker MQTT quando o programa for encerrado
# client.loop_stop()
# client.disconnect()