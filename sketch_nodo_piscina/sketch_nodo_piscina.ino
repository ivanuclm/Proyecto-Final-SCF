#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <NewPing.h>


// Pines y configuración
#define DHT_PIN 15
#define POTENTIOMETER_PIN 13
const int servoPin = 18;
int potValue = 0;
const int UltrasonicPin = 5;
const int MaxDistance = 200;

NewPing sonar(UltrasonicPin, UltrasonicPin, MaxDistance);

DHTesp dhtSensor;
Servo servo;

const char* ssid = "MiFibra-4B1E";
const char* password = "QF6oM3q7";

#define TOPIC_DHT "/sensors/dht11"
#define TOPIC_US "/sensors/ultrasonic"

// Configuración del servidor MQTT
const char* mqtt_server = "192.168.1.19";
const int mqtt_port = 1885;
const char* mqtt_user = "admin";
const char* mqtt_password = "admin";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Función de conexión WiFi
void wifiConnect() {
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();  // Guarda el tiempo de inicio
  const unsigned long wifiTimeout = 1000000;   // 10 segundos de límite

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startAttemptTime >= wifiTimeout) {
      Serial.println("No se pudo conectar al WiFi. Reiniciando...");
      ESP.restart();  // Reinicia el ESP32 si no se conecta
    }
    delay(500);
    Serial.println(WiFi.status());
  }

  Serial.println("\nConectado a WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Función de conexión MQTT
void mqttConnect() {
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32_Client", mqtt_user, mqtt_password)) {
      Serial.println("Connected to MQTT!");
      client.subscribe(TOPIC_DHT);
      client.subscribe(TOPIC_US);
    } else {
      Serial.print("MQTT connection failed, state: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// Tarea para leer datos del sensor y publicar en MQTT
void taskDHT(void* parameter) {
  while (true) {
    TempAndHumidity data = dhtSensor.getTempAndHumidity();

    if (!isnan(data.temperature) && !isnan(data.humidity)) {
      // Crear JSON
      StaticJsonDocument<128> jsonDoc;
      jsonDoc["value"] = data.temperature;
      

      // Serializar JSON
      char jsonBuffer[128];
      serializeJson(jsonDoc, jsonBuffer);

      // Publicar en MQTT
      client.publish(TOPIC_DHT, jsonBuffer);

      // Mostrar en Monitor Serie
      Serial.println("JSON Sent:");
      Serial.println(jsonBuffer);
    } else {
      Serial.println("Error reading DHT sensor.");
    }

    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

// Tarea para controlar el servo motor
void taskServo(void* parameter) {
  int pos = 0;

  while (true) {
    // Mover el servo de 0 a 180 grados
    for (pos = 0; pos <= 180; pos++) {
      servo.write(pos);
      vTaskDelay(15 / portTICK_PERIOD_MS);  // Pausa entre movimientos
    }

    // Mover el servo de 180 a 0 grados
    for (pos = 180; pos >= 0; pos--) {
      servo.write(pos);
      vTaskDelay(15 / portTICK_PERIOD_MS);  // Pausa entre movimientos
    }
  }
}

// Tarea para leer datos del potenciometro que simula el ph
void taskPotenph(void* parameter) {
  while (true) {
    int potValue = analogRead(POTENTIOMETER_PIN);
    float voltage = (potValue / 4095.0) * 3.3;
    int percentage = map(potValue, 0, 4095, 0, 100);

    Serial.print("Raw Value: ");
    Serial.print(potValue);
    Serial.print(" | Voltage: ");
    Serial.print(voltage);
    Serial.print(" V | Percentage: ");
    Serial.print(percentage);
    Serial.println(" %");

    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Leer cada 500ms
  }
}

// Tarea para mantener la conexión WiFi y MQTT
void taskWiFiMQTT(void* parameter) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      wifiConnect();
    }

    if (!client.connected()) {
      Serial.println("MQTT disconnected, reconnecting...");
      mqttConnect();
    }

    client.loop();  // Procesar mensajes MQTT
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Revisar cada segundo
  }
}

void taskUltrasonido(void* parameter) {  
  while(true) {
    float cm = sonar.ping_cm();
    Serial.print(cm); // obtener el valor en cm (0 = fuera de rango)
    Serial.println("cm");
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    if (!isnan(cm)) {
      // Crear JSON
      StaticJsonDocument<128> jsonDoc;
      jsonDoc["value"] = cm;
      

      // Serializar JSON
      char jsonBuffer[128];
      serializeJson(jsonDoc, jsonBuffer);

      // Publicar en MQTT
      client.publish(TOPIC_US, jsonBuffer);

      // Mostrar en Monitor Serie
      Serial.println("JSON Sent:");
      Serial.println(jsonBuffer);
    } else {
      Serial.println("Error reading US sensor.");
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configurar sensor DHT y servo
  dhtSensor.setup(DHT_PIN, DHTesp::DHT11);
  servo.attach(servoPin, 500, 2400);
  pinMode(POTENTIOMETER_PIN, INPUT);

  // Conexión inicial a WiFi y MQTT
  wifiConnect();
  mqttConnect();

  // Crear las tareas de FreeRTOS
  xTaskCreate(taskDHT, "Task DHT", 4096, NULL, 1, NULL);
  xTaskCreate(taskServo, "Task Servo", 4096, NULL, 1, NULL);
  xTaskCreate(taskWiFiMQTT, "Task WiFi/MQTT", 4096, NULL, 1, NULL);
  xTaskCreate(taskPotenph, "Task Potentiometer ph", 2048, NULL, 1, NULL);
  xTaskCreate(taskUltrasonido, "Task Distancia Agua", 4096, NULL, 1, NULL);
}

void loop() {
  
}
