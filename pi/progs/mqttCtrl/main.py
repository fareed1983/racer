import time    #https://docs.python.org/fr/3/library/time.html
import paho.mqtt.client as mqtt
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/

c = 14
s = 15

n = 92
m = 90

pca = ServoKit(channels=16)

print('starting...')
pca.servo[c].angle = n
pca.servo[s].angle = m
time.sleep(5)
print('init done')


pca.servo[c].set_pulse_width_range(500, 2500)
pca.servo[s].set_pulse_width_range(500, 2500)

#broker_address = "192.168.56.49"
broker_address = "localhost"
port = 1883
topic = "racer/+/C"

def on_connect(client, userdata, flags, rc, properties):
    print("Connected with result code " + str(rc))
    client.subscribe(topic)

def on_message(client, userdata, msg):
    mstr = str(msg.payload.decode('utf-8'))
    print(f"Rcv: {msg.topic} {mstr}")
    client.publish(msg.topic[:-1] + "M", mstr)
    if msg.topic.startswith("racer/E/C"):
        ang = n + int(mstr)
        if ang > n-20 and ang < n+40:
            pca.servo[c].angle = ang
        else:
            print("invalid value")
    if msg.topic.startswith("racer/S/C"):
        ang = m + int(mstr)
        if ang > m-60 and ang < m+60:
            pca.servo[s].angle = ang
        else:
            print("invalid value")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address, port=port)

client.loop_forever()
