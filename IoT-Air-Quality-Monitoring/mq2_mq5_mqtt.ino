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
#define MQTT_PUB_MQ2 "esp32/mq2"
#define MQTT_PUB_MQ5 "esp32/mq5"


//MQ5
#define sensor 34 //35

//MQ2//
#define Gas_analog 32 
#define Gas_digital 33  


// Variables to hold sensor readings
int gassensorAnalog;
int gassensorDigital;

int gas_value;

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
  pinMode(Gas_digital, INPUT); //MQ2
  
  pinMode(sensor, INPUT); //MQ5
  Serial.println();
  
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

    //MQ2//
    gassensorAnalog = analogRead(Gas_analog);
    gassensorDigital = digitalRead(Gas_digital);
  
    Serial.print("Gas Sensor: ");
    Serial.print(gassensorAnalog);
    Serial.print("\t");
  
    if (gassensorAnalog > 1000) {
      Serial.println("Gas");
    }
    else {
      Serial.println("No Gas");
    }
    
    //MQ5//
    gas_value = analogRead(sensor);
    Serial.print("Gas Value:");
    Serial.println(gas_value);

    if (gas_value > 1000) {
      Serial.println("Gas");
    }
    else {
      Serial.println("No Gas");
    }
    
    // Check if any reads failed and exit early (to try again).
    if (isnan(gassensorAnalog) || isnan(gas_value)) {
      Serial.println(F("Failed to read from sensor!"));
      return;
    }
    
    // Publish an MQTT message on topic esp32/mq2
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_MQ2, 1, true, String(gassensorAnalog).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_MQ2, packetIdPub1);
    Serial.printf("Message: %.2f \n", gassensorAnalog);

    // Publish an MQTT message on topic esp32/mq5
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_MQ5, 1, true, String(gas_value).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_MQ5, packetIdPub4);
    Serial.printf("Message: %.2f \n", gas_value);
  }
}
