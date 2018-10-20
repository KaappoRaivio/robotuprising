import tty

import paho.mqtt.client as mqtt
import time

import sys

import termios
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
TIME  = 1


fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    ch = sys.stdin.read(1)
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def cleanup():
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)



def _onPress(key):
    if not isinstance(key, str):
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys
    else:
        k = key

    if k == "d":
        print(k)
        client.publish("local", f"x {STEP} y 0.0 rot 0.0 dt {TIME}")
    elif k == 'w':
        print(k)
        client.publish("local", f"x 0.0 y {STEP} rot 0.0 dt {TIME}")
    elif k == 's':
        print(k)
        client.publish("local", f"x 0.0 y -{STEP} rot 0.0 dt {TIME}")
    elif k == 'a':
        print(k)
        client.publish("local", f"x -{STEP} y 0.0 rot 0.0 dt {TIME}")
    elif k == "q":
        client.publish("local", f"x 0.0 y 0.0 rot {ROT_STEP} dt {TIME}")
    elif k == "e":
        client.publish("local", f"x 0.0 y 0.0 rot -{ROT_STEP} dt {TIME}")
    elif k == " ":
        client.publish("local", "manual_stop")
    elif k == "r":
        client.publish("local", "grab")
    elif k == "f":
        client.publish("local", "release")

    # print(f"{key} pressed!")

    return True


def _onRelease(key):
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys

    # print(f"{key} released!")

    return True


# listener = keyboard.Listener(on_press=_onPress, on_release=_onRelease)
# listener.start()
try:
    while True:
        key = getch()
        _onPress(key)
finally:
    cleanup()