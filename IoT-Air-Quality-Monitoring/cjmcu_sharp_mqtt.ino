#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "YOUR_WIFI_SSID" 
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"  

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(35, 180, 23, 120)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// MQTT Topics
#define MQTT_PUB_CJMCU4541  "esp32/cjmcu4541"
#define MQTT_PUB_SHARPGP2Y10  "esp32/sharpgp2y10"


//CJMCU4541
#define PRE_PIN          32   
#define VNOX_PIN         35   
#define VRED_PIN         34   

#define PRE_HEAT_SECONDS 10


//SHARPGP2Y10
#define measurePin  33 
#define ledPower  2   

//CJMCU4541
int vnox_value;
int vred_value;

//SHARPGP2Y10
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured;
float calcVoltage;
float dustDensity;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time sensors was published
const long interval = 10000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PRE_PIN, OUTPUT); //CJMCU4541
    
  digitalWrite(PRE_PIN, 1);
  delay(PRE_HEAT_SECONDS * 1000);
  digitalWrite(PRE_PIN, 0);
  Serial.println("Done");

  pinMode(ledPower,OUTPUT); //SHARPGP2Y10
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    //CJMCU4541//
    vnox_value = analogRead(VNOX_PIN);
    vred_value = analogRead(VRED_PIN);

    //SHARPGP2Y10//
    digitalWrite(ledPower,LOW); // power on the LED
    delayMicroseconds(samplingTime);
    voMeasured = analogRead(measurePin); // read the dust value

    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);
    
    // 0 - 5V mapped to 0 - 1023 integer values
    // recover voltage
    calcVoltage = voMeasured * (5.0 / 1024.0);
  
    // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
    // Chris Nafis (c) 2012
    dustDensity = 170 * calcVoltage - 0.1;
    
    Serial.print("Vnox: ");
    Serial.print(vnox_value, DEC);
    Serial.print(" Vred: ");
    Serial.println(vred_value, DEC);

    if (vred_value > 1000) {
      Serial.println("Gas");
    }
    else {
      Serial.println("No Gas");
    }
    
    Serial.print("Dust: ");
    Serial.println(dustDensity); // unit: ug/m3

 
    // Check if any reads failed and exit early (to try again).
    if (isnan(vred_value) || isnan(dustDensity)) {
      Serial.println(F("Failed to read from sensor!"));
      return;
    }

    // Publish an MQTT message on topic esp32/cjmcu4541
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_CJMCU4541, 1, true, String(vred_value).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_CJMCU4541, packetIdPub2);
    Serial.printf("Message: %.2f \n", vred_value);
    
    // Publish an MQTT message on topic esp32/sharpgp2y10
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_SHARPGP2Y10, 1, true, String(dustDensity).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_SHARPGP2Y10, packetIdPub3);
    Serial.printf("Message: %.2f \n", dustDensity);
  }
}
