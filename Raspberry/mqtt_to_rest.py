import paho.mqtt.client as mqtt
import requests
import psycopg2
from psycopg2 import Error

# Configuración MQTT
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_REQUEST = "quimicos/request"

# Configuración REST API
REST_API_URL = "http://172.20.10.2:8080/quimicos/{}"

# Configuración PostgreSQL
DB_HOST = "localhost"
DB_NAME = "quimicos_db"
DB_USER = "pi"
DB_PASSWORD = "raspberry"

def init_db():
    """Inicializa la base de datos PostgreSQL y crea la tabla chemicals."""
    try:
        conn = psycopg2.connect(
            host=DB_HOST,
            database=DB_NAME,
            user=DB_USER,
            password=DB_PASSWORD
        )
        cursor = conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chemicals (
                id SERIAL PRIMARY KEY,
                nombre_quimico VARCHAR(255) NOT NULL,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        conn.commit()
        print(" Base de datos PostgreSQL inicializada para chemicals.")
    except Error as e:
        print(f" Error inicializando la base de datos: {e}")
    finally:
        if conn:
            cursor.close()
            conn.close()

def insert_chemical(nombre_quimico):
    """Inserta un nombre de químico en la base de datos PostgreSQL."""
    try:
        conn = psycopg2.connect(
            host=DB_HOST,
            database=DB_NAME,
            user=DB_USER,
            password=DB_PASSWORD
        )
        cursor = conn.cursor()
        cursor.execute(
            "INSERT INTO chemicals (nombre_quimico) VALUES (%s)",
            (nombre_quimico,)
        )
        conn.commit()
        print(f" Químico insertado: {nombre_quimico}")
    except Error as e:
        print(f" Error insertando químico: {e}")
    finally:
        if conn:
            cursor.close()
            conn.close()

def send_rest_request(nombre_quimico):
    """Envía una solicitud REST a la API externa."""
    try:
        # Codificar el nombre del químico para la URL
        import urllib.parse
        encoded_chemical = urllib.parse.quote(nombre_quimico)
        url = REST_API_URL.format(encoded_chemical)
        response = requests.get(url, timeout=5)
        if response.status_code == 200:
            print(f"Solicitud REST exitosa: {response.json()}")
        else:
            print(f" Solicitud REST fallida con código: {response.status_code}")
    except Exception as e:
        print(f" Error enviando solicitud REST: {e}")

def on_connect(client, userdata, flags, rc):
    print(f"Conectado al broker MQTT con código: {rc}")
    client.subscribe(MQTT_TOPIC_REQUEST)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"Solicitud química recibida: {payload}")
    send_rest_request(payload)
    insert_chemical(payload)

# Inicializar la base de datos
init_db()

# Configurar y empezar el cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_forever()