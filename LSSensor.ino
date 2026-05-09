#include <EspDrv.h>
#include <MQTTClient.h>
#include <SoftwareSerial.h>
#include "config.h"
#include "secret.h"
#include <avr/wdt.h>

#define LSSensorPIN1 2
#define LSSensorPIN2 3
#define SENDINTERVAL 5 * 60 * 1000 //5 minut

void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length) { }
void OnBusy(uint8_t count);
void DataTimeout();
MQTTConnectData mqttConnectData = { MQTTHost, 1883, "WattMeter", MQTTUsername, MQTTPassword, "", 0, false, "", false, 0x0F }; 

SoftwareSerial serial(4, 5);
EspDrv espDrv(&serial);
MQTTClient mqttClient(&espDrv, MQTTMessageReceive);
char data[32];
int wattMetter1Counter = 0;
int wattMetter2Counter = 0;
unsigned long lastSendToMQTT = 0;
unsigned long lastTime = 0;
bool closeRequired = false;

#pragma pack(push, 1)
struct DiagData {
  uint32_t uptime;
  uint16_t freeRam;
  uint16_t wifiReconn;
  uint16_t mqttFailCount;
  uint8_t  resetReason;
  uint16_t loopMaxMs;
};
#pragma pack(pop)

DiagData currentDiagData;

extern int __heap_start, *__brkval;
int freeRam() {
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

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
  int wifiStatus = espDrv.GetConnectionStatus();
  bool wifiConnected = wifiStatus == WL_CONNECTED;
  if(wifiStatus == WL_DISCONNECTED || wifiStatus == WL_IDLE_STATUS)
  {
    wifiConnected = espDrv.Connect(WifiSSID, WifiPassword);
    if(currentDiagData.wifiReconn < 65535) 
    {
      currentDiagData.wifiReconn++;
    }
  }
  if(wifiConnected)
  {

    bool isConnected = mqttClient.IsConnected();
    if(!isConnected)
    {
      bool ok = mqttClient.Connect(mqttConnectData);
      if(!ok && currentDiagData.mqttFailCount < 65535) currentDiagData.mqttFailCount++;
      return ok;
    }
    else
    {
      return true;
    }
  }
  return false;
}

void DataTimeout()
{
  closeRequired = true;
}
void OnBusy(uint8_t count)
{
  if(count > 10)
  {
    closeRequired = true;
  }
}

void setup() {
  currentDiagData.resetReason = MCUSR;
  MCUSR = 0;
  // put your setup code here, to run once:
  pinMode(LSSensorPIN1, INPUT);
  pinMode(LSSensorPIN2, INPUT);
  Serial.begin(57600);
  serial.begin(57600);
  espDrv.Init(16);
  espDrv.OnBusy = OnBusy;
  espDrv.DataTimeout = DataTimeout;
  espDrv.Connect(WifiSSID, WifiPassword);
  attachInterrupt(digitalPinToInterrupt(LSSensorPIN1), WattMetter1Received, RISING);
  attachInterrupt(digitalPinToInterrupt(LSSensorPIN2), WattMetter2Received, RISING);
  Serial.println("Setup OK");
  wdt_enable(WDTO_8S);
}

void loop() {
  unsigned long currentMillis = millis();
  wdt_reset();
  if(closeRequired)
  {
    espDrv.Close();
    closeRequired = false;
  }
  mqttClient.Loop();
  if(currentMillis - lastSendToMQTT >= 300000)
  {    
    detachInterrupt(digitalPinToInterrupt(LSSensorPIN1));
    detachInterrupt(digitalPinToInterrupt(LSSensorPIN2));
    if(Connect())
    {
      sprintf(data, "{\"V\":%d,\"S\":%d}", 0, 0);
      mqttClient.Publish(ELCONSUMPTION, data);
      sprintf(data, "{\"V\":%d,\"S\":%d}", wattMetter1Counter, wattMetter2Counter);
      mqttClient.Publish(ELCONSUMPTION, data);
      mqttClient.Disconnect();
      
      wattMetter1Counter = 0;
      wattMetter2Counter = 0;

      currentDiagData.uptime = currentMillis / 60000UL;
      currentDiagData.freeRam = freeRam();
      mqttClient.Publish(LSSENSOR_DIAG, (const uint8_t*)&currentDiagData, sizeof(DiagData), false);
      currentDiagData.loopMaxMs = 0;
    }
    lastSendToMQTT = currentMillis;
    attachInterrupt(digitalPinToInterrupt(LSSensorPIN1), WattMetter1Received, RISING);
    attachInterrupt(digitalPinToInterrupt(LSSensorPIN2), WattMetter2Received, RISING);
  }
  unsigned long iterDur = millis() - currentMillis;
  if(iterDur > currentDiagData.loopMaxMs) 
  {
    currentDiagData.loopMaxMs = (iterDur > 65535UL) ? 65535 : (uint16_t)iterDur;
  }
}
