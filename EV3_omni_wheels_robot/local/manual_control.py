import paho.mqtt.client as mqtt
import time
from pynput import keyboard

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("$SYS/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

def send_message(text):
    client.publish("local", text)






client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("iot.eclipse.org", 1883, 60)
#client.connect("170.252.127.33", 1883, 60)
#client.connect("localhost", 1883, 60)


# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
#client.publish("topic/test", "Hello world!")


handle = open("bl_input.txt")    
text = handle.read()

STEP = 100.0
ROT_STEP = 30.0

def _onPress(key):
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys

    if k == "d":
        print(k)
        client.publish("local", f"x {STEP} y 0.0 rot 0.0 dt 1.0")
    elif k == 'w':
        print(k)
        client.publish("local", f"x 0.0 y {STEP} rot 0.0 dt 1.0")
    elif k == 's':
        print(k)
        client.publish("local", f"x 0.0 y -{STEP} rot 0.0 dt 1.0")
    elif k == 'a':
        print(k)
        client.publish("local", f"x -{STEP} y 0.0 rot 0.0 dt 1.0")
    elif k == "q":
        client.publish("local", f"x 0.0 y 0.0 rot {ROT_STEP} dt 1.0")
    elif k == "e":
        client.publish("local", f"x 0.0 y 0.0 rot -{ROT_STEP} dt 1.0")

    # print(f"{key} pressed!")

    return True


def _onRelease(key):
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys

    # print(f"{key} released!")

    return True


listener = keyboard.Listener(on_press=_onPress, on_release=_onRelease)
listener.start()

while True:
    pass