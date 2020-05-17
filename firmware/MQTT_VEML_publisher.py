import os
import json
from mintsXU4 import mintsDefinitions as mD
import paho.mqtt.client as mqtt
import time 

# DEFINE PATH TO VEML SENSOR .json
mac_address = mD.findMacAddress()
json_path = mD.dataFolder + '/' + mac_address+ '/VEML6075.json'

# DEFINE TOPIC TO WHICH WE'LL PUBLISH SENSOR DATA 
topic = 'vision60/VEML_sensor' 

# BROKER PARAMETERS
broker_address = "192.168.0.13"
port = 1883

if __name__ == '__main__':
     print("creating new instance...")
     ourClient = mqtt.Client("Vision60") # Create a MQTT client object

     print("connecting to broker...")
     ourClient.connect(broker_address,port) # Connect to the MQTT broker
     
     # CONTINUOUSLY PUBLISH SENSOR DATA
     print("publishing VEML sensor data...")
     while(True):
          # OPEN .json 
          with open(json_path) as file:
               data = json.load(file) 
               rawUVA =float(data['rawUVA'])
               
               #Publish sensor data
               ourClient.publish(topic,rawUVA)

          time.sleep(4) # wait