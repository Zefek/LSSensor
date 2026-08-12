#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "time.h"
#include "esp_sntp.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "config.h"
#include "secret.h"
#include "ota.h"

#ifndef FW_VERSION
#define FW_VERSION 0
#endif
#define LSSensorPIN1 32
#define LSSensorPIN2 33
#define SAMPLE_INTERVAL_MS 5
#define ADC_TH_LOW_MV 1000
#define ADC_TH_HIGH_MV 1800
#define PULSE_MIN_INTERVAL_MS 30
#define SAMPLER_CORE 1
#define SAMPLER_PRIORITY 2
#define SAMPLER_STACK 4096
#define SAMPLER_REPORT_ITERS 2000
#define PIN_DIAG_INTERVAL_MS 500UL
#define PULSE_INTERVAL 60000UL
#define DIAG_INTERVAL 300000UL
#define WDT_TIMEOUT_S 90
#define WIFI_CONNECT_TIMEOUT_MS 15000UL
#define TIME_SYNC_TIMEOUT_MS 15000UL
#define MQTT_TLS_PORT 8883
#define TIME_VALID_THRESHOLD 1700000000UL
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"

WiFiClientSecure net;
PubSubClient mqtt(net);

volatile uint32_t counter1 = 0;
volatile uint32_t counter2 = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

bool chHigh1 = false;
bool chHigh2 = false;
uint32_t chLastCount1 = 0;
uint32_t chLastCount2 = 0;

uint32_t samplerMaxUs = 0;
uint32_t samplerOverruns = 0;
uint16_t samplerStackWords = 0;

char data[32];
unsigned long lastPulseSend = 0;
unsigned long lastDiagSend = 0;
unsigned long lastPinDiag = 0;
bool initialZeroSent = false;
bool timeSynced = false;
char ntpFromDhcp[16] = "";

#pragma pack(push, 1)
struct DiagData {
  uint32_t uptime;
  uint16_t freeRamKb;
  uint16_t wifiReconn;
  uint16_t mqttFailCount;
  uint8_t  resetReason;
  uint16_t loopMaxMs;
  int8_t   rssi;
  uint16_t fwVersion;
  uint16_t otaFailCount;
  uint16_t samplerMaxUs;
  uint16_t samplerStackWords;
  uint16_t samplerOverruns;
};
#pragma pack(pop)
static_assert(sizeof(DiagData) == 24, "DiagData wire layout must stay 24 bytes");

DiagData currentDiagData;
uint16_t otaFailures = 0;

uint16_t heapKb()
{
  return (uint16_t)(ESP.getFreeHeap() / 1024);
}

void ProcessSample(int mv, bool* high, uint32_t* lastCount, volatile uint32_t* counter, uint32_t now)
{
  if(*high)
  {
    if(mv < ADC_TH_LOW_MV)
    {
      *high = false;
    }
  }
  else if(mv > ADC_TH_HIGH_MV)
  {
    *high = true;
    if(now - *lastCount > PULSE_MIN_INTERVAL_MS)
    {
      portENTER_CRITICAL(&mux);
      (*counter)++;
      portEXIT_CRITICAL(&mux);
      *lastCount = now;
    }
  }
}

void SamplerTask(void* arg)
{
  TickType_t last = xTaskGetTickCount();
  chHigh1 = analogReadMilliVolts(LSSensorPIN1) > ADC_TH_HIGH_MV;
  chHigh2 = analogReadMilliVolts(LSSensorPIN2) > ADC_TH_HIGH_MV;
  uint32_t iterCount = 0;
  uint32_t durSum = 0;
  uint32_t durMax = 0;
  uint32_t allTimeMaxUs = 0;
  uint32_t overrunCount = 0;
  for(;;)
  {
    int64_t start = esp_timer_get_time();
    uint32_t now = millis();
    int mv1 = analogReadMilliVolts(LSSensorPIN1);
    int mv2 = analogReadMilliVolts(LSSensorPIN2);
    ProcessSample(mv1, &chHigh1, &chLastCount1, &counter1, now);
    ProcessSample(mv2, &chHigh2, &chLastCount2, &counter2, now);
    uint32_t dur = (uint32_t)(esp_timer_get_time() - start);

    durSum += dur;
    if(dur > durMax)
    {
      durMax = dur;
    }
    if(dur > allTimeMaxUs)
    {
      allTimeMaxUs = dur;
    }
    if(dur > SAMPLE_INTERVAL_MS * 1000UL)
    {
      overrunCount++;
      Serial.printf("SAMPLER overrun: %lu us\n", (unsigned long)dur);
    }

    if(++iterCount >= SAMPLER_REPORT_ITERS)
    {
      uint16_t hwm = (uint16_t)uxTaskGetStackHighWaterMark(NULL);
      uint32_t avg = durSum / iterCount;
      portENTER_CRITICAL(&mux);
      samplerMaxUs = allTimeMaxUs;
      samplerStackWords = hwm;
      samplerOverruns = overrunCount;
      portEXIT_CRITICAL(&mux);
      Serial.printf("SAMPLER avg=%lu us max=%lu us stackHWM=%u words overruns=%lu\n",
        (unsigned long)avg, (unsigned long)durMax, hwm, (unsigned long)overrunCount);
      iterCount = 0;
      durSum = 0;
      durMax = 0;
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
  }
}

bool SyncTime()
{
  const ip_addr_t* dhcpServer = esp_sntp_getserver(0);
  if(dhcpServer != NULL && !ip_addr_isany_val(*dhcpServer))
  {
    snprintf(ntpFromDhcp, sizeof(ntpFromDhcp), "%s", ipaddr_ntoa(dhcpServer));
  }
  if(ntpFromDhcp[0] != '\0')
  {
    Serial.printf("NTP z DHCP: %s\n", ntpFromDhcp);
    configTime(0, 0, ntpFromDhcp, NTP_SERVER_1, NTP_SERVER_2);
  }
  else
  {
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
  }
  unsigned long start = millis();
  time_t now = time(nullptr);
  while(now < TIME_VALID_THRESHOLD && millis() - start < TIME_SYNC_TIMEOUT_MS)
  {
    esp_task_wdt_reset();
    delay(200);
    now = time(nullptr);
  }
  return now >= TIME_VALID_THRESHOLD;
}

bool Connect()
{
  if(WiFi.status() != WL_CONNECTED)
  {
    if(currentDiagData.wifiReconn < 65535)
    {
      currentDiagData.wifiReconn++;
    }
    WiFi.begin(WifiSSID, WifiPassword);
    unsigned long start = millis();
    while(WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS)
    {
      esp_task_wdt_reset();
      delay(100);
    }
  }
  if(WiFi.status() != WL_CONNECTED)
  {
    return false;
  }
  if(!timeSynced)
  {
    timeSynced = SyncTime();
    if(!timeSynced)
    {
      return false;
    }
  }
  if(!mqtt.connected())
  {
    bool ok = mqtt.connect("WattMeter", MQTTUsername, MQTTPassword);
    if(!ok && currentDiagData.mqttFailCount < 65535)
    {
      currentDiagData.mqttFailCount++;
    }
    return ok;
  }
  return true;
}

void setup() {
  currentDiagData.resetReason = (uint8_t)esp_reset_reason();
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  esp_sntp_servermode_dhcp(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WifiSSID, WifiPassword);
  net.setCACert(MQTTCACert);
  mqtt.setServer(MQTTHost, MQTT_TLS_PORT);
  mqtt.setBufferSize(256);
  mqtt.setKeepAlive(60);
  xTaskCreatePinnedToCore(SamplerTask, "sampler", SAMPLER_STACK, NULL, SAMPLER_PRIORITY, NULL, SAMPLER_CORE);
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&wdtConfig);
  esp_task_wdt_add(NULL);
  Serial.println("Setup OK");
}

void loop() {
  unsigned long currentMillis = millis();
  esp_task_wdt_reset();
  mqtt.loop();

  bool pulseDue = !initialZeroSent || currentMillis - lastPulseSend >= PULSE_INTERVAL;
  bool diagDue = currentMillis - lastDiagSend >= DIAG_INTERVAL;

  if(pulseDue || diagDue)
  {
    portENTER_CRITICAL(&mux);
    uint32_t c1 = counter1;
    uint32_t c2 = counter2;
    uint32_t sMaxUs = samplerMaxUs;
    uint16_t sStackWords = samplerStackWords;
    uint32_t sOverruns = samplerOverruns;
    portEXIT_CRITICAL(&mux);

    bool connected = Connect();

    if(pulseDue)
    {
      if(connected)
      {
        if(!initialZeroSent)
        {
          sprintf(data, "{\"V\":%lu,\"S\":%lu}", 0UL, 0UL);
          mqtt.publish(ELCONSUMPTION, data);
          initialZeroSent = true;
        }
        sprintf(data, "{\"V\":%lu,\"S\":%lu}", (unsigned long)c1, (unsigned long)c2);
        mqtt.publish(ELCONSUMPTION, data);
      }
      lastPulseSend = currentMillis;
    }

    if(diagDue)
    {
      if(connected)
      {
        currentDiagData.uptime = currentMillis / 60000UL;
        currentDiagData.freeRamKb = heapKb();
        currentDiagData.rssi = (int8_t)WiFi.RSSI();
        currentDiagData.fwVersion = (uint16_t)FW_VERSION;
        currentDiagData.otaFailCount = otaFailures;
        currentDiagData.samplerMaxUs = (sMaxUs > 65535UL) ? 65535 : (uint16_t)sMaxUs;
        currentDiagData.samplerStackWords = sStackWords;
        currentDiagData.samplerOverruns = (sOverruns > 65535UL) ? 65535 : (uint16_t)sOverruns;
        mqtt.publish(LSSENSOR_DIAG, (const uint8_t*)&currentDiagData, sizeof(DiagData), false);
        currentDiagData.loopMaxMs = 0;
      }
      lastDiagSend = currentMillis;
    }
  }

  otaLoop();

  unsigned long iterDur = millis() - currentMillis;
  if(iterDur > currentDiagData.loopMaxMs)
  {
    currentDiagData.loopMaxMs = (iterDur > 65535UL) ? 65535 : (uint16_t)iterDur;
  }
}
