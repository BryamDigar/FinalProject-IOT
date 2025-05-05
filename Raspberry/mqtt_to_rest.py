import paho.mqtt.client as mqtt
import requests

# MQTT Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "quimicos/request"

# REST API Configuration
REST_API_URL = "http://localhost:8080/quimicos/{}"

# Callback for MQTT connection
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with code: {rc}")
    client.subscribe(MQTT_TOPIC)

# Callback for MQTT message
def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"Received MQTT message: {payload}")
    send_rest_request(payload)

# Send REST request
def send_rest_request(nombre_quimico):
    try:
        response = requests.get(REST_API_URL.format(nombre_quimico))
        if response.status_code == 200:
            print(f"REST request successful: {response.json()}")
        else:
            print(f"REST request failed with status code: {response.status_code}")
    except Exception as e:
        print(f"Error sending REST request: {e}")

# Configure MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to MQTT broker
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Keep the MQTT client running
client.loop_forever()
