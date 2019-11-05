#include <Arduino.h>
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}

#include <AsyncMqttClient.h>

#define WIFI_SSID "utbm_visiteurs"
#define WIFI_PASSWORD ""

#define MQTT_HOST IPAddress(172, 17, 6, 1)
#define MQTT_PORT 1883

#define RELAY_TOPIC "test/relay"
#define RELAY1 12
#define RELAY2 13
#define RELAY3 14
#define RELAY4 15
#define RELAY5 27


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

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
  uint16_t packetIdSub = mqttClient.subscribe(RELAY_TOPIC, 2); // Subscribe with Qos 2
  Serial.print("Subscribed to ");
  Serial.println(RELAY_TOPIC);

  //uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  //Serial.print("Publishing at QoS 2, packetId: ");
  //Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  if(strcmp(topic, RELAY_TOPIC) == 0 ) {
    int pin;
    if(messageTemp == "1") {
      pin = RELAY1;
    }
    else if(messageTemp == "2") {
      pin = RELAY2;
    }
    else if(messageTemp == "3") {
      pin = RELAY3;
    }
    else if(messageTemp == "4") {
      pin = RELAY4;
    }
    else if(messageTemp == "5") {
      pin = RELAY5;
    }

    Serial.print("Pin activated : ");
    Serial.println(pin);
    digitalWrite(pin, LOW);
    delay(300);
    digitalWrite(pin, HIGH);
  }

  Serial.print("Publish on ");
  Serial.print(topic);
  Serial.print(" => ");
  Serial.println(messageTemp);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(9600);
  while (! Serial);

  Serial.println();
  Serial.println();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  digitalWrite(RELAY3, HIGH);
  digitalWrite(RELAY4, HIGH);
  digitalWrite(RELAY5, HIGH);
}

void loop() {
}
