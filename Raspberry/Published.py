import paho.mqtt.client as mqtt
import psycopg2
from psycopg2 import Error
import json
import time

# Configuración MQTT
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_DATA = "quimicos/data"
MQTT_TOPIC_ALERT = "quimicos/alert"
MQTT_TOPIC_CONTROL = "quimicos/control"

# Configuración PostgreSQL
DB_HOST = "localhost"
DB_NAME = "quimicos_db"
DB_USER = "pi"
DB_PASSWORD = "raspberry"

def init_db():
    """Inicializa la base de datos PostgreSQL y crea la tabla sensor_data."""
    try:
        conn = psycopg2.connect(
            host=DB_HOST,
            database=DB_NAME,
            user=DB_USER,
            password=DB_PASSWORD
        )
        cursor = conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS sensor_data (
                id SERIAL PRIMARY KEY,
                temp REAL,
                co REAL,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        conn.commit()
        print(" Base de datos PostgreSQL inicializada para sensor_data.")
    except Error as e:
        print(f"Error inicializando la base de datos: {e}")
    finally:
        if conn:
            cursor.close()
            conn.close()

def insert_sensor_data(temp, co):
    """Inserta datos de sensores en la base de datos PostgreSQL."""
    try:
        conn = psycopg2.connect(
            host=DB_HOST,
            database=DB_NAME,
            user=DB_USER,
            password=DB_PASSWORD
        )
        cursor = conn.cursor()
        cursor.execute(
            "INSERT INTO sensor_data (temp, co) VALUES (%s, %s)",
            (temp, co)
        )
        conn.commit()
        print(f" Datos de sensores insertados: temp={temp}, co={co}")
    except Error as e:
        print(f" Error insertando datos de sensores: {e}")
    finally:
        if conn:
            cursor.close()
            conn.close()

def publish_alert(client, door_status, air_status):
    """Publica doorStatus y airStatus en el tópico quimicos/alert."""
    alert_data = {
        "doorStatus": door_status,
        "airStatus": air_status,
        "timestamp": int(time.time())
    }
    alert_message = json.dumps(alert_data)
    result = client.publish(MQTT_TOPIC_ALERT, alert_message)
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f" Alerta publicada en {MQTT_TOPIC_ALERT}: {alert_message}")
    else:
        print(f" Error publicando alerta en {MQTT_TOPIC_ALERT}")

def on_connect(client, userdata, flags, rc):
    print(f" Conectado al broker MQTT con código: {rc}")
    client.subscribe(MQTT_TOPIC_DATA)
    client.subscribe(MQTT_TOPIC_ALERT)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    topic = msg.topic
    print(f"Mensaje recibido en el tópico '{topic}': {payload}")

    if topic == MQTT_TOPIC_DATA:
        try:
            data = json.loads(payload)
            # Insertar temp y co en la base de datos
            insert_sensor_data(data["temp"], data["co"])
            # Publicar doorStatus y airStatus en quimicos/alert
            if "doorStatus" in data and "airStatus" in data:
                publish_alert(client, data["doorStatus"], data["airStatus"])
        except json.JSONDecodeError as e:
            print(f" Error decodificando JSON: {e}")
        except KeyError as e:
            print(f" Falta un campo en los datos: {e}")
    elif topic == MQTT_TOPIC_ALERT:
        print(f"Alerta recibida: {payload}")
        # Aquí se puede implementar lógica adicional para manejar alertas

# Inicializar la base de datos
init_db()

# Configurar y empezar el cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_forever()