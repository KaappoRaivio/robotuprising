import paho.mqtt.client as mqtt
import time

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
#client.connect("localhost", 1883, 60)
#client.connect("170.252.127.33", 1883, 60)
#client.connect("localhost", 1883, 60)


# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
#client.publish("topic/test", "Hello world!")

input_files = ["./movingdata/2.0-rods.txt"]
#input_files = ["./movingdata/justwait.txt"]

for f in input_files:

    handle = open(f)    
    text = handle.read()

    for line in text.split("\n"):

        client.publish("local", line)
        print("MQTT published: ", line)
        time.sleep( float(line.split(" ")[-1]) )


#client.loop_forever()
