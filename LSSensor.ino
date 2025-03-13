#include <EspDrv.h>
#include <MQTTClient.h>
#include <SoftwareSerial.h>
#include "config.h"
#include <avr/wdt.h>

#define LSSensorPIN1 2
#define SENDINTERVAL 5 * 60 * 1000 //5 minut

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length) { }
MQTTConnectData mqttConnectData = { MQTTHost, 1883, "WattMeter", MQTTUsername, MQTTPassword, "", 0, false, "", false, 0x0 }; 

SoftwareSerial serial(4, 5);
EspDrv espDrv(&serial);
MQTTClient mqttClient(&espDrv, MQTTMessageReceive);
char data[5];
char data2[5];

int wattMetter1Counter = 0;
int wattMetter2Counter = 0;
unsigned long lastSendToMQTT = 0;

unsigned long lastTime = 0;
void WattMetter1Received()
{
  unsigned long time = millis();
  if(time - lastTime > 84)
  {
    wattMetter1Counter++;
    lastTime = time;
  }
}

unsigned long lastTime2 = 0;
void WattMetter2Received()
{
  unsigned long time = millis();
  if(time - lastTime2 > 84)
  {
    wattMetter2Counter++;
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
      return mqttClient.Connect(mqttConnectData);
    }
    else
    {
      return true;
    }
  }
  return false;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  Serial.begin(74880);
  serial.begin(74880);
  espDrv.Init(16);
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
    detachInterrupt(digitalPinToInterrupt(2));
    detachInterrupt(digitalPinToInterrupt(3));
    Serial.println(wattMetter1Counter);
    Serial.println(wattMetter2Counter);
    sprintf(data, "%d", wattMetter1Counter);
    sprintf(data2, "%d", wattMetter2Counter);
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
