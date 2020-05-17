import os
import json
from mintsXU4 import mintsDefinitions as mD
import paho.mqtt.client as mqtt
import time 

# DEFINE TOPIC FROM WHICH WE'LL RECIEVE SENSOR DATA 
topic = 'vision60/VEML_sensor'

# BROKER PARAMETERS
broker_address = "192.168.0.13"
port = 1883

def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)

if __name__ == '__main__':
    print("creating new instance...")
    ourClient = mqtt.Client("VEML_sub") # Create a MQTT client object

    print("connecting to broker...")
    ourClient.connect(broker_address, port) # Connect to the MQTT broker
    
    # SUBSCRIBE TO SENSOR TOPIC
    print("Subscribing to topic",topic)
    ourClient.subscribe(topic)
    ourClient.on_message = on_message # attach function to callback 

    # RUN MQTT SUBSCRIBER INDEFINITELY   
    while(True):
        ourClient.loop_start() #start the loop
        time.sleep(4) # wait