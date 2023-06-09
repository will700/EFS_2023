import paho.mqtt.client as mqtt, paho.mqtt.publish as publish
import subprocess
import my_module
import time

# MQTT_SERVER = "192.168.3.8"  # will 4b
MQTT_SERVER = "192.168.3.5"  # EFS 3A
# MQTT_SERVER = "192.168.0.22" # home 3a
# MQTT_SERVER = "10.204.64.150" # uav rpi 4b

TOPIC = "EFS/target_pos"
var1 = '1'
trial = '0'
output1 = 0
# Connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    publish.single(TOPIC, "BSS connected", hostname=MQTT_SERVER)
    client.subscribe(TOPIC)

# Run script when message received
def on_message(client, userdata, msg):
    global var1, var2,trial
    start = time.time()
    var2 = msg.payload.decode()
    subprocess.call(["python3","/home/drone3/Desktop/itp/batt_swap.py",var1,var2])
    output1=my_module.position_check() 
    trial = '0'
    start = time.time()
    while(output1 == None and var2 != var1): 
        output1=my_module.position_check()
        subprocess.call(["python3","/home/drone3/Desktop/itp/bss_align.py",trial])    
        trial = int(trial)+1
        trial = str(trial)  
    end = time.time() 
    print(end - start)            
    var1 = var2
    end = time.time()
    publish.single("EFS/current_pos", var1, hostname=MQTT_SERVER)
    
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_SERVER, 1883, 60)

client.loop_forever()
