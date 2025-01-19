#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include "DHTesp.h"

// Pines y configuración
#define DHT_PIN 15
const int servoPin = 18;

DHTesp dhtSensor;
Servo servo;

const char* ssid = "MiFibra-4B1E";
const char* password = "QF6oM3q7";

#define TOPIC_TEMP "/sensors/water_temperature"
#define TOPIC_HUM "/sensors/water_level"

// Configuración del servidor MQTT
const char* mqtt_server = "192.168.1.20";
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
      client.subscribe(TOPIC_TEMP);
      client.subscribe(TOPIC_HUM);
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
      // Convertir los valores a cadenas
      char tempString[8];
      char humString[8];
      dtostrf(data.temperature, 1, 2, tempString);
      dtostrf(data.humidity, 1, 2, humString);

      // Publicar los datos en MQTT
      client.publish(TOPIC_TEMP, tempString);
      client.publish(TOPIC_HUM, humString);

      // Mostrar en el monitor serie
      Serial.print("Temperature: ");
      Serial.println(tempString);
      Serial.print("Humidity: ");
      Serial.println(humString);
    } else {
      Serial.println("Error reading DHT sensor.");
    }

    // Delay para evitar saturar el broker MQTT
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

void setup() {
  Serial.begin(115200);

  // Configurar sensor DHT y servo
  dhtSensor.setup(DHT_PIN, DHTesp::DHT11);
  servo.attach(servoPin, 500, 2400);

  // Conexión inicial a WiFi y MQTT
  wifiConnect();
  mqttConnect();

  // Crear las tareas de FreeRTOS
  xTaskCreate(taskDHT, "Task DHT", 4096, NULL, 1, NULL);
  xTaskCreate(taskServo, "Task Servo", 4096, NULL, 1, NULL);
  xTaskCreate(taskWiFiMQTT, "Task WiFi/MQTT", 4096, NULL, 1, NULL);
}

void loop() {
  // El loop queda vacío, todo se maneja con FreeRTOS
}
