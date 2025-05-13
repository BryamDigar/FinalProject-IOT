#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MQUnifiedsensor.h>
#include <WiFi.h>
#include "WebServer.h"
#include "WebPage.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Definición de pines y constantes
#define DHTPIN 4
#define DHTTYPE DHT11
#define LED_PIN 27
#define BUZZER_PIN 26
#define FLAME_PIN 25
#define Board "ESP-32"
#define Pin 34  // Usamos un pin ADC adecuado para el ESP32

#define Type "MQ-2"
#define Voltage_Resolution 3.3
#define ADC_Bit_Resolution 12 
#define RatioMQ2CleanAir 9.83

#define echoPin0 2
#define trigPin0 15

#define echoPin1 12
#define trigPin1 14

#define fanPin 32

#define RED_PIN 5
#define GREEN_PIN 18
#define BLUE_PIN 19

#define TEMP_LOW 8.4
#define TEMP_HIGH 13
#define ROOM_SIZE 50
#define DOOR_PROX 5
#define SAFE_THRES 50
#define DANGER_THRES 120
#define FIRE_THRESHOLD LOW

#define I2C_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_LINES 2

#define HISTORY_SIZE 60 // Guardar 60 lecturas (5 minutos con lecturas cada 5 segundos)

// Inicialización del servidor web
WebServer server(HTTP_PORT);

const char* mqtt_server = "172.20.10.2";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_Fire_Detector";
const char* topic_data = "quimicos/data";
const char* topic_alert = "quimicos/alert";
const char* topic_request = "quimicos/request";
const char* topic_control = "quimicos/control";

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

// set up del HC-SR04
long duration0, distance0;
long duration1, distance1;

// Variables para almacenar lecturas de sensores
const int numReadings = 10;
float tempReadings[numReadings] = {0};
float humiReadings[numReadings] = {0};
int coReadings[numReadings] = {0};
float dispReadings[numReadings] = {0};
int index_gas = 0;

QueueHandle_t sensorDataQueue;

volatile bool alarmTriggered = false;
volatile bool dataReady = false;

// Variables para histórico de datos
float tempHistory[HISTORY_SIZE] = {0};
float humiHistory[HISTORY_SIZE] = {0};
int coHistory[HISTORY_SIZE] = {0};
String doorStatusHistory[HISTORY_SIZE] = {""};
String airStatusHistory[HISTORY_SIZE] = {""};
unsigned long timestampHistory[HISTORY_SIZE] = {0};
int historyIndex = 0;
unsigned long lastHistoryUpdate = 0;
const unsigned long historyInterval = 5000; // Actualizar cada 5 segundos

// Variables para notificaciones y alarmas
volatile bool alarmActive = false;
String notifications[5] = {"", "", "", "", ""};
int notificationCount = 0;

struct SensorData {
  float temp;
  float humi;
  int co;
  float disp;
  float disp1;
};

WiFiClient espClient;
PubSubClient client(espClient);

SensorData sensorData;

byte Alert0[8] = {0b00001, 0b00010, 0b00101, 0b00101, 0b01000, 0b10001, 0b10000, 0b01111};
byte Alert1[8] = {0b00000, 0b10000, 0b01000, 0b01000, 0b00100, 0b00010, 0b00010, 0b11100};

void IRAM_ATTR triggerAlarm() {
    alarmTriggered = true;
}

// Publicar datos vía MQTT
void publishData(SensorData data) {
  StaticJsonDocument<200> doc;
  doc["temp"] = data.temp;
  doc["co"] = data.co;
  doc["timestamp"] = millis();
  
  char buffer[256];
  serializeJson(doc, buffer);
  if (client.publish(topic_data, buffer)) {
    Serial.println("Datos publicados a MQTT: " + String(buffer));
  } else {
    Serial.println("Error al publicar datos a MQTT");
  }
}

// Publicar alerta vía MQTT
void publishAlert(String doorStatus, String airStatus) {
  StaticJsonDocument<200> docAlert;
  docAlert["doorStatus"] = doorStatus;
  docAlert["airStatus"] = airStatus;
  
  char buffer0[256];
  serializeJson(docAlert, buffer0);
  if (client.publish(topic_alert, buffer0)) {
    Serial.println("Alerta publicada a MQTT: " + String(buffer0));
  } else {
    Serial.println("Error al publicar alerta a MQTT");
  }
}

// Callback de MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Mensaje recibido en topic " + String(topic) + ": " + message);
  if (String(topic) == topic_control && message == "disable_alarm") {
    alarmTriggered = false;
    alarmActive = false;
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
    addNotification("Alarma desactivada remotamente");
  }
}

// Reconectar al broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando al broker MQTT...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("Conectado");
      client.subscribe(topic_control);
    } else {
      Serial.println("Fallo, reintentando en 5 segundos");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

// Tarea para leer los sensores periódicamente
void readSensorsTask(void *pvParameters) {
  while (true) {
    sensorData.temp = dht.readTemperature();
    MQ2.update();
    sensorData.co = analogRead(Pin);
    sensorData.co = map(sensorData.co, 0, 4095, 0, 100);
    digitalWrite(trigPin0, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin0, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin0, LOW);
    duration0 = pulseIn(echoPin0, HIGH);
    distance0 = duration0 / 58.2;
    sensorData.disp = distance0;

    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    duration1 = pulseIn(echoPin1, HIGH);
    distance1 = duration1 / 58.2;
    sensorData.disp1 = distance1;

    xQueueSend(sensorDataQueue, &sensorData, portMAX_DELAY);
    dataReady = true;
    vTaskDelay(500 / portTICK_PERIOD_MS); // Leer sensores cada 500ms
  }
}

// Manejar la solicitud raíz del servidor web
void handleRoot() {
  server.send_P(200, "text/html", MAIN_page);
}

// Manejar las solicitudes de datos del servidor web
void handleData() {
  String doorStatus = "Puerta Cerrada";
  String airStatus = "Ambiente Seguro";
  if (sensorData.temp > TEMP_HIGH) airStatus = "PERÓXIDOS ORGÁNICOS (Clase 5.2)";
  else if (sensorData.temp < TEMP_LOW) airStatus = "SUSTANCIAS TÓXICAS (Clase 6.1)";
  else if (DANGER_THRES > sensorData.co > SAFE_THRES) airStatus = "LÍQUIDOS INFLAMABLES (Clase 3)";
  else if (sensorData.co > DANGER_THRES) airStatus = "SUSTANCIAS QUE PUEDEN EXPERIMENTAR COMBUSTIÓN ESPONTÁNEA (Clase 4.2)";
  else if (sensorData.disp < ROOM_SIZE && sensorData.disp1 <= DOOR_PROX) doorStatus = "Movimiento Detectado";
  else if (sensorData.disp1 > DOOR_PROX) airStatus = "Puerta Abierta";
  else if (alarmActive) airStatus = "SUSTANCIAS COMBURENTES (Clase 5.1)";

  String json = "{";
  json += "\"temp\":" + String(sensorData.temp) + ",";
  json += "\"co\":" + String(sensorData.co) + ",";
  json += "\"doorStatus\":" + doorStatus + ",";
  json += "\"airStatus\":\"" + airStatus + "\",";
  json += "\"alarmActive\":" + String(alarmActive ? "true" : "false") + ",";
  json += "\"lcd\":\"" + airStatus + "\\nTemp: " + String(sensorData.temp) + " %\\nMQ2 = " + String(sensorData.co) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleHistory() {
  String json = "[";
  int count = 0;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    int idx = (historyIndex - i - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    if (timestampHistory[idx] == 0) continue;
    if (count > 0) json += ",";
    json += "{";
    json += "\"timestamp\":" + String(timestampHistory[idx]) + ",";
    json += "\"temp\":" + String(tempHistory[idx]) + ",";
    json += "\"co\":" + String(coHistory[idx]) + ",";
    json += "\"doorStatus\":" + doorStatusHistory[idx] + ",";
    json += "\"airStatus\":\"" + airStatusHistory[idx] + "\"";
    json += "}";
    count++;
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleNotifications() {
  String json = "[";
  for (int i = 0; i < notificationCount; i++) {
    if (i > 0) json += ",";
    json += "\"" + notifications[i] + "\"";
  }
  json += "]";
  server.send(200, "application/json", json);
}

// Manejar la solicitud para desactivar la alarma
void handleDisableAlarm() {
  alarmTriggered = false;
  alarmActive = false;
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);
  addNotification("Alarma desactivada manualmente");
  server.send(200, "text/plain", "Alarma desactivada");
}

// Agregar una notificación al historial
void addNotification(String message) {
  for (int i = 4; i > 0; i--) notifications[i] = notifications[i-1];
  unsigned long currentTime = millis();
  String timeString = String(currentTime / 60000) + "m " + String((currentTime / 1000) % 60) + "s";
  notifications[0] = "[" + timeString + "] " + message;
  if (notificationCount < 5) notificationCount++;
}

// Agregar datos al historial
void addToHistory(float temp, int co, String airStatus, String doorStatus) {
  tempHistory[historyIndex] = temp;
  coHistory[historyIndex] = co;
  doorStatusHistory[historyIndex] = doorStatus;
  airStatusHistory[historyIndex] = airStatus;
  timestampHistory[historyIndex] = millis();
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
}

void setRGB(int red, int green, int blue) {
  analogWrite(RED_PIN, 255 - red);
  analogWrite(GREEN_PIN, 255 - green);
  analogWrite(BLUE_PIN, 255 - blue);
}

void setFan(boolean state){
  if (state){
    digitalWrite(fanPin, HIGH);
  } else {
    digitalWrite(fanPin, LOW);
  }
}

// Bucle principal del programa
void myLoopTask( void *pvParameters) {
  SensorData receivedData;
  while(true){
    server.handleClient();

    if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
      dataReady = false;

      // Mostrar datos en el LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Distancia int:");
      lcd.setCursor(0, 1);
      lcd.print(sensorData.disp);
      lcd.print(" cm");
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Puerta:");
      lcd.setCursor(0, 1);
      lcd.print(sensorData.disp1);
      lcd.print(" cm");
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Monitoreo OK");
      lcd.setCursor(0, 1);
      lcd.print("Temp: ");
      lcd.print(sensorData.temp);
      lcd.print("'C");
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Monitoreo OK");
      lcd.setCursor(0, 1);
      lcd.print("MQ2 = ");
      lcd.print(sensorData.co);
      delay(1000);
      lcd.clear();
      Serial.println("Temp: " + String(sensorData.temp) + " CO: " + String(sensorData.co) + "Distancia interna" + String(sensorData.disp) + "Distancia Puerta"+ String(sensorData.disp1));

      // Actualizar lecturas de sensores
      tempReadings[index_gas] = sensorData.temp;
      coReadings[index_gas] = sensorData.co;
      index_gas = (index_gas + 1) % numReadings;

      float tempDiff = tempReadings[index_gas] - tempReadings[(index_gas + 1) % numReadings];
      int coDiff = coReadings[index_gas] - coReadings[(index_gas + 1) % numReadings];

      // Detectar condiciones de alarma
      bool fireDetected = digitalRead(FLAME_PIN) == FIRE_THRESHOLD;
      bool highTemp = sensorData.temp > TEMP_HIGH;
      bool lowTemp = sensorData.temp < TEMP_LOW;
      bool highCO = sensorData.co > SAFE_THRES;
      bool rapidChange = (tempDiff > 2) || (coDiff > 5);

      String doorStatus = "Puerta Cerrada";
      String airStatus = "Ambiente Seguro";
      if (sensorData.disp1 > DOOR_PROX){
        doorStatus = "Puerta Abierta";
      } else if (sensorData.disp < ROOM_SIZE && sensorData.disp1 <= DOOR_PROX) {
        doorStatus = "Movimiento detectado";
      } if (highTemp) {
        airStatus = "PERÓXIDOS ORGÁNICOS (Clase 5.2)";
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Monitoreo OK");
        lcd.setCursor(0, 1);
        lcd.print("Temp alta");
        setFan(true);
        lcd.setCursor(14,1);
        lcd.write(byte(0));
        lcd.setCursor(15,1);
        lcd.write(byte(1));
        delay(1000);
        setRGB(255, 0, 0);
        lcd.clear();
      } else if (lowTemp) {
        airStatus = "SUSTANCIAS TÓXICAS (Clase 6.1)";
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Monitoreo OK");
        lcd.setCursor(0, 1);
        lcd.print("Temp baja");
        lcd.setCursor(14,1);
        lcd.write(byte(0));
        lcd.setCursor(15,1);
        lcd.write(byte(1));
        delay(1000);
        setRGB(255, 0, 0);
        lcd.clear();  
      } else if (highCO){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Encendiendo");
        lcd.setCursor(0, 1);
        lcd.print("Ventilador");
        setFan(true);
        delay(1000);
      } else{
        setRGB(0, 255, 0);
        setFan(false);
        digitalWrite(LED_PIN, LOW);
      }

      // Activar alarma si se detectan condiciones de incendio
      if (fireDetected || (highTemp) || (rapidChange && highCO)) {
        triggerAlarm();
      }

      // Manejar la activación de la alarma
      if (alarmTriggered && !alarmActive) {
        alarmActive = true;
        addNotification("¡ALARMA DE PRECAUCIÓN!");
        tone(BUZZER_PIN, 1000);
        digitalWrite(LED_PIN, HIGH);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Alerta!!");
        delay(1000);
      }

      if (alarmActive) {
        airStatus = "SUSTANCIAS COMBURENTES (Clase 5.1)";
      }

      // Actualizar el historial de datos
      if (millis() - lastHistoryUpdate >= historyInterval) {
        addToHistory(sensorData.temp, sensorData.co, airStatus, doorStatus);
        lastHistoryUpdate = millis();
      }

      publishData(receivedData);
      publishAlert(doorStatus, airStatus);
      
    }
    vTaskDelay(100/ portTICK_PERIOD_MS);
  }
}

// Configuración inicial del sistema
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(trigPin0, OUTPUT);
  pinMode(echoPin0, INPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(fanPin, OUTPUT);

  sensorDataQueue = xQueueCreate(5, sizeof(SensorData));
  if (sensorDataQueue == NULL) {
  Serial.println("Error al crear la cola.");
  }

  MQ2.setRegressionMethod(1);
  MQ2.setA(36974); 
  MQ2.setB(-3.109);
  MQ2.init(); 

  lcd.init();
  lcd.backlight();
  lcd.setCursor(4, 0);
  lcd.print("----*----");
  lcd.setCursor(2, 1);
  lcd.print("Alarm System");
  delay(1000);
  lcd.clear();

  Serial.print("Calibrating MQ2...");
  float calcR0 = 0;
  for(int i = 1; i <= 10; i++) {
    MQ2.update();
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("\nCalibration complete. R0 = " + String(calcR0 / 10));

  attachInterrupt(digitalPinToInterrupt(FLAME_PIN), triggerAlarm, FALLING);
  lcd.createChar(0, Alert0);
  lcd.createChar(1, Alert1);

  // Conexión a WiFi
  Serial.println("Conectando a WiFi...");
  lcd.setCursor(0, 0);
  lcd.print("Conectando a");
  lcd.setCursor(0, 1);
  lcd.print("WiFi...");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    WiFi.begin(SSID, PASSWORD);
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConectado a WiFi! IP: " + WiFi.localIP().toString());
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Conectado");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(2000);
  } else {
    Serial.println("\nFallo al conectar a WiFi.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Fallo WiFi");
    while (true) delay(1000);
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Configuración de rutas del servidor web
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/history", handleHistory);
  server.on("/notifications", handleNotifications);
  server.on("/disableAlarm", handleDisableAlarm);
  server.begin();
  Serial.println("Servidor web iniciado.");

  // Crear tarea para leer sensores
  xTaskCreate(readSensorsTask, "ReadSensorsTask", 2048, NULL, 1, NULL);
  xTaskCreate(myLoopTask, "LoopTask", 4096, NULL, 1, NULL);
}
void loop(){
  if (!client.connected()){
    reconnect();
  }
  client.loop();
}