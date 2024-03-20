# ====================================================================
# Implementacao do client MQTT utilizando a bilbioteca MQTT Paho
# ====================================================================
import paho.mqtt.client as mqtt
import time
import random
import rospy
from mav import MAV2  # Seu arquivo MAV2.py com a classe MAV2
import time

user = ""
passwd = ""

Broker = "192.168.6.78"            # Endereco do broker
Port = 1883                           # Porta utilizada 
KeepAlive = 60                      # Intervalo de timeout (60s)
TopicoL = "out"               # Topico que sera lido
TopicoE = "in"              # Topico que sera escrito
client_id = f'python-mqtt-{random.randint(0, 1000)}'

db = 1                              # Flag de depuracao (verbose)
z = 2
square_size = 1.5
# Quando conectar na rede (Callback de conexao)
def on_connect(client, userdata, flags, rc):
    print("Conectado com codigo " + str(rc))
    client.subscribe(TopicoL, qos=0)

# Quando receber uma mensagem (Callback de mensagem)
def on_message(client, userdata, msg):
    client.newmsg = True    
    client.msg = msg.payload.decode("utf-8")

# Funcao que espera nova mensagem do terminal
# Por enquanto o parametro "topico" nao possui utilidade
def le_terminal(topico=TopicoL, verbose=db):
    client.newmsg = False

    # Fica em loop infinito ate receber uma nova mensagem
    while not client.newmsg:
        client.loop_start()
        # time.sleep(2)
        client.loop_stop()

    if verbose == 1:
        print("> " + client.msg)
        if client.msg == '1':
            try:
                # dr.go_to_local([-square_size, 0, z])  # Isso publicará um comando para mover 1 metro no eixo X
                dr.set_vel(2,0,0)
            except KeyboardInterrupt:
                print("foi")
        else:
            print("> " + client.msg)
            try:
                # dr.go_to_local([-square_size, 0, z])  # Isso publicará um comando para mover 1 metro no eixo X
                dr.set_vel(0,0,0)
            except KeyboardInterrupt:
                print("foi")

    return client.msg

# Funcao que escreve no terminal
# Por padrao vai escrever no topico apontado por "TopicoE"
def escreve_terminal(frase, topico=TopicoE, verbose=db):
    client.publish(topico, payload=frase, qos=0, retain=False)
    if verbose == 1:
        print(frase)

rospy.init_node('mqtt_to_ros_bridge')
dr = MAV2()
try:
    dr.takeoff(2)
    print("Takeoff sent")
except KeyboardInterrupt:
        print("foi")
        
client = mqtt.Client()                      # Criacao do cliente MQTT
client.on_connect = on_connect              # Vinculo do Callback de conexao
client.on_message = on_message              # Vinculo do Callback de mensagem recebida
# client.username_pw_set(user, passwd)        # Apenas para coneccao com login/senha
client.connect(Broker, Port, KeepAlive)     # Conexao do cliente ao broker
# escreve_terminal('1')
while True:
    le_terminal()