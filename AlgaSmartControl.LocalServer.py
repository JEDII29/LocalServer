import json
import os
import requests
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from signalr import Connection
from datetime import datetime

class Central:
    def __init__(self):
        self.temperatures = {'Comfort': 22, 'Standard': 20, 'Economic': 18}
        self.schedule = {}  # Format: {'HH:MM': 'TemperatureType'}

class Zone:
    def __init__(self, id, manual_temp=None):
        self.id = id
        self.manual_temp = manual_temp
        self.current_temp = None
        self.automatic = True
        self.heating = False
        self.devices = []

    def update_temperature(self, temp):
        self.current_temp = temp
        self.evaluate_heating()

    def evaluate_heating(self):
        if self.automatic:
            # Logic for automatic temperature management
            pass
        else:
            self.heating = self.manual_temp > self.current_temp
        for device in self.devices:
            device.set_state(self.heating)

class Device:
    def __init__(self, id, gpio_pin):
        self.id = id
        self.gpio_pin = gpio_pin
        self.active = False
        GPIO.setup(gpio_pin, GPIO.OUT)

    def set_state(self, state):
        self.active = state
        GPIO.output(self.gpio_pin, GPIO.HIGH if state else GPIO.LOW)

# Initialization
central = Central()
zones = {}
devices = {}

# MQTT Configuration
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("home/zone/+/temperature")

def on_message(client, userdata, msg):
    try:
        topic_parts = msg.topic.split('/')
        zone_id = topic_parts[2]
        temperature = float(msg.payload.decode())
        if zone_id in zones:
            zones[zone_id].update_temperature(temperature)
    except Exception as e:
        print(f"Failed to process message: {e}")

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

# SignalR Client Configuration
class SignalRHandler:
    def __init__(self, url, hub_name):
        self.connection = Connection(url, session=None)
        self.hub = self.connection.register_hub(hub_name)
        self.connection.start()

    def send_temperature_update(self, zone_id, temperature):
        self.hub.server.invoke("SendTemperatureUpdate", {"zone_id": zone_id, "temperature": temperature})

signalr_handler = SignalRHandler('http://central-server/signalr', 'CentralHub')

# GPIO Setup
GPIO.setmode(GPIO.BCM)

# Function to register central unit with central server
def register_central_unit():
    mac_address = 'B8:27:EB:XX:XX:XX'  # Example MAC address
    registration_data = {
        'email': 'user@example.com',
        'password': 'password123',
        'mac_address': mac_address,
        'devices': [{'id': 'device1', 'type': 'pump', 'gpio_pin': 17}]
    }
    response = requests.post('http://central-server/register', json=registration_data)
    print(response.json())

# Example usage
if __name__ == '__main__':
    # Initial setup
    zone1 = Zone('zone1')
    device1 = Device('device1', 17)
    zone1.devices.append(device1)
    zones['zone1'] = zone1
    devices['device1'] = device1

    # Registering central unit with central server
    register_central_unit()

    # Main loop to periodically send temperature updates
    try:
        while True:
            for zone_id, zone in zones.items():
                if zone.current_temp is not None:
                    signalr_handler.send_temperature_update(zone_id, zone.current_temp)
            time.sleep(60)
    except KeyboardInterrupt:
        GPIO.cleanup()
        mqtt_client.disconnect()