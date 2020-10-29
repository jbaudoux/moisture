// You need to create a secret.h file with your WiFi credentials.
#include "secret.h"
// #define WIFI_SSID "myssid"
// #define WIFI_PASSWORD "*********"
// Set password to "" for open networks.


//#include <Wire.h>
#include <Adafruit_ADS1015.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// hardware fixed pins
// WakeUp on Timer: D0=RST
// SCL=GPIO4=D1
// SDA=GPIO5=D2

Adafruit_ADS1115 ads1115(0x48);

const char* mqttServer = "192.168.10.1";
const int mqttPort = 1883;
const char* clientId = "sprinkler/sensor/moisture/01";
//const char* mqttUser = "YourMqttUser";
//const char* mqttPassword = "YourMqttUserPassword";

WiFiClient espClient;
PubSubClient client(espClient);

void bubbleSort(int16_t a[], int size) {
  for(int i=0; i<(size-1); i++) {
    for(int o=0; o<(size-(i+1)); o++) {
      if(a[o] > a[o+1]) {
        int16_t t = a[o];
        a[o] = a[o+1];
        a[o+1] = t;
      }
    }
  }
}

void setup(void)
{
  unsigned long now = millis();
  unsigned long period;
  Serial.begin(9600);
  Serial.println("Hello!");

  Serial.print("Connecting to WiFi ssid=");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int count = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    count += 1;
    if (count > 20) {
      Serial.println("");
      Serial.println("Give up trying to connect. Going to deepSleep for 1 minutes");
      period = millis() - now;
      ESP.deepSleep(1 * 60 * 1000000 - period*1000); // deepSleep time is defined in microseconds.
    }
  }
  Serial.print("Connected to the WiFi with IP=");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);
  Serial.println("Connecting to MQTT");
  count = 0;
  while (!client.connected()) {
    if (client.connect(clientId)) { //, mqttUser, mqttPassword )) { 
      Serial.print("Connected to MQTT as ");
      Serial.println(clientId);
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
      count += 1;
      if (count > 3) {
        Serial.println("");
        Serial.println("Give up trying to connect. Going to deepSleep for 5 minutes");
        period = millis() - now;
        ESP.deepSleep(5 * 60 * 1000000 - period*1000); // deepSleep time is defined in microseconds.
      }
      delay(2000);
    }
  }

  // Power-up the ADC and sensors = D4
  pinMode (D4, OUTPUT);
  digitalWrite(D4, HIGH);
  Serial.println("Powering up ADC");
  delay(500);
  
  Serial.println("Getting single-ended readings from AIN0..3");
  ads1115.setGain(GAIN_ONE);
  Serial.println("ADC Range: +/-4.096V");
  ads1115.begin();

  // Make 5 readings and take median
  int16_t adc0[5], adc1[5], adc2[5], adc3[5];
  for (count = 0; count < 5; count++) {
    delay(500); // First, wait sensor initialisation
    adc0[count] = ads1115.readADC_SingleEnded(0);
    adc1[count] = ads1115.readADC_SingleEnded(1);
    adc2[count] = ads1115.readADC_SingleEnded(2);
    adc3[count] = ads1115.readADC_SingleEnded(3);
  }

  // Power down ADC and sensors
  digitalWrite(D4, LOW);

  // Sort to find median
  Serial.println("Sorting");
  bubbleSort(adc0, 5);
  bubbleSort(adc1, 5);
  bubbleSort(adc2, 5);
  bubbleSort(adc3, 5);

  Serial.println("Computing percentage");
  short adc0pc, adc1pc, adc2pc, adc3pc;

  adc0pc = (short) (27000 - adc0[2]) / 270;
  adc1pc = (short) (27000 - adc1[2]) / 270;
  adc2pc = (short) (27000 - adc2[2]) / 270;
  adc3pc = (short) (27000 - adc3[2]) / 270;

  float volt = 0;//(float) adc1 / 5424;
  
  Serial.print("AIN0: "); Serial.print(adc0[2]); Serial.print(" %"); Serial.println(adc0pc);
  Serial.print("AIN1: "); Serial.print(adc1[2]); Serial.print(" %"); Serial.println(adc1pc);
  Serial.print("AIN2: "); Serial.print(adc2[2]); Serial.print(" %"); Serial.println(adc2pc);
  Serial.print("AIN3: "); Serial.print(adc3[2]); Serial.print(" %"); Serial.println(adc3pc);
  //Serial.print("AIN1: "); Serial.print(adc1); Serial.print(" "); Serial.print(volt); Serial.println("Volts");
  Serial.println(" ");

  Serial.print("Publishing values...");
  StaticJsonDocument<300> msg;
  msg["moisture01"] = adc0pc;
  msg["moisture02"] = adc1pc;
  msg["moisture03"] = adc2pc;
  msg["moisture04"] = adc3pc;
  //msg["battery"] = volt;
  char JSONmessageBuffer[200];
  serializeJson(msg, JSONmessageBuffer);

  if (client.publish(clientId, JSONmessageBuffer, true)) {
    Serial.println(" done");
  } else {
    Serial.println(" error");
  }
  
  client.disconnect();
  
  delay(1000);
  Serial.println("Going to deepSleep");
  period = millis() - now;
  ESP.deepSleep(1 * 60 * 1000000 - period*1000); // deepSleep time is defined in microseconds.
}

void loop(void)
{
}
