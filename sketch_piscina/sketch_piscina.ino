#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <NewPing.h>
#include "credentials.h"


// Pines y configuración
#define DHT_PIN 15
#define POTENTIOMETER_PIN 35
const int servoPin = 18;
int potValue = 0;
const int UltrasonicPin = 5;
const int MaxDistance = 200;

NewPing sonar(UltrasonicPin, UltrasonicPin, MaxDistance);

DHTesp dhtSensor;
Servo servo;



#define TOPIC_DHT "/sensors/dht11"
#define TOPIC_US "/sensors/ultrasonic"
#define TOPIC_POT "/sensors/potentiometer"

// Configuración del servidor MQTT
const char* mqtt_server = "192.168.1.19";
const int mqtt_port = 1885;
const char* mqtt_user = "admin";
const char* mqtt_password = "admin";
float water_level = 100;
float phValue = 7;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void IRAM_ATTR handleButtonPress() {
    buttonPressed = true;
}

// Funcion para enviar valores a traves de MQTT
void publishMQTT(char* topic, float value){
  if (!isnan(value)) {
      // Crear JSON
      StaticJsonDocument<128> jsonDoc;
      jsonDoc["value"] = value;

      // Serializar JSON
      char jsonBuffer[128];
      serializeJson(jsonDoc, jsonBuffer);

      // Publicar en MQTT
      client.publish(topic, jsonBuffer);

      // Mostrar en Monitor Serie
      Serial.println("JSON Sent:");
      Serial.println(jsonBuffer);
    } else {
      Serial.println("Error publishing MQTT.");
    }
}

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
    publishMQTT(TOPIC_DHT, data.temperature);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

// Tarea para controlar el servo motor
void taskServo(void* parameter) {
  int pos = 0;  // Posición inicial del servo

  while (true) {
    if (water_level < 90 && pos < 180) { // Si el nivel de agua es menor a 90% y la posición del servo es menor a 180 (cerrado)
      pos++;
      servo.write(pos); //vamos abriendo la valvula
      vTaskDelay(15 / portTICK_PERIOD_MS);
    } else if (water_level > 95 && pos > 0) { // Si el nivel de agua es mayor a 95% y la posición del servo es mayor a 0 (abierto)
      pos--;
      servo.write(pos);
      vTaskDelay(15 / portTICK_PERIOD_MS); //vamos cerrando la valvula
    }
  }
}

// Tarea para leer datos del potenciometro que simula el ph
void taskPotenph(void* parameter) {
  while (true) {
    int potValue = analogRead(POTENTIOMETER_PIN);
    float phValue = ( potValue / 4095.0 ) * 14.0;

    Serial.print("Ph Actual: ");
    Serial.println(phValue);
    publishMQTT(TOPIC_POT, phValue);
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
    float cm = sonar.ping_cm() - 2; // restamos la distancia minima que mide el sensor 
    float pool_height = 2 * 100; // 2 metros a centimetros
    water_level =  ((pool_height - cm) / pool_height) * 100; // calculamos capacidad actual
    Serial.print(water_level); // obtener el valor en cm (0 = fuera de rango)
    Serial.println("% water_level");
    if (water_level > 0) {
      publishMQTT(TOPIC_US, water_level);
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);
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
  if (buttonPressed) {
        buttonPressed = false;
        Serial.println("Sistema interrumpido");
        // Acciones específicas
    }
}
