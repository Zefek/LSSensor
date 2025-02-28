#include <EspDrv.h>
#include <MQTTClient.h>
#include <SoftwareSerial.h>
#include "config.h"
#include<avr/wdt.h>

#define LSSensorPIN1 2
#define SENDINTERVAL 5 * 60 * 1000 //5 minut

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length) { }

SoftwareSerial serial(4, 5);
EspDrv espDrv(&serial);
MQTTClient mqttClient(&espDrv, MQTTMessageReceive);
char data[5];
char data2[5];

float wattMetter1Counter = 0;
float wattMetter2Counter = 0;
unsigned long lastSendToMQTT = 0;

unsigned long lastTime = 0;
void WattMetter1Received()
{
  unsigned long time = millis();
  if(time - lastTime > 84)
  {
    wattMetter1Counter += 0.1;
    lastTime = time;
  }
}

unsigned long lastTime2 = 0;
void WattMetter2Received()
{
  unsigned long time = millis();
  if(time - lastTime2 > 84)
  {
    wattMetter2Counter += 0.1;
    lastTime2 = time;
  }
}

bool Connect()
{
  uint8_t wifiStatus = espDrv.GetConnectionStatus();
  bool wifiConnected = wifiStatus == WL_CONNECTED;
  if(wifiStatus == WL_DISCONNECTED || wifiStatus == WL_IDLE_STATUS)
  {
    wifiConnected = espDrv.Connect(WifiSSID, WifiPassword);
  }
  if(wifiConnected)
  {
    uint8_t clientStatus = espDrv.GetClientStatus();
    if(clientStatus == CL_DISCONNECTED)
    {
      return mqttClient.Connect(MQTTHost, 1883, "WattMeter", MQTTUsername, MQTTPassword, "", 0, false, "", false);
    }
  }
  return false;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  Serial.begin(57600);
  serial.begin(57600);
  espDrv.Init();
  espDrv.Connect(WifiSSID, WifiPassword);
  attachInterrupt(digitalPinToInterrupt(2), WattMetter1Received, RISING);
  attachInterrupt(digitalPinToInterrupt(3), WattMetter2Received, RISING);
  Serial.println("Setup OK");
  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();
  unsigned long currentMillis = millis();
  if(currentMillis - lastSendToMQTT >= 300000)
  {    
    Serial.println(wattMetter1Counter);
    Serial.println(wattMetter2Counter);
    detachInterrupt(digitalPinToInterrupt(2));
    detachInterrupt(digitalPinToInterrupt(3));
    sprintf(data, "%d", (int)round(wattMetter1Counter * 10));
    sprintf(data2, "%d", (int)round(wattMetter2Counter * 10));
    if(Connect())
    {
      mqttClient.Publish(ELVRCH, "0");
      mqttClient.Publish(ELVRCH, data);
      mqttClient.Publish(ELSPODEK, "0");
      mqttClient.Publish(ELSPODEK, data2);
      mqttClient.Disconnect();
      
      wattMetter1Counter = 0;
      wattMetter2Counter = 0;
    }
    lastSendToMQTT = currentMillis;
    attachInterrupt(digitalPinToInterrupt(2), WattMetter1Received, RISING);
    attachInterrupt(digitalPinToInterrupt(3), WattMetter2Received, RISING);
  }
}
