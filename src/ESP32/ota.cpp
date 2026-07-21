#include "ota.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <esp_task_wdt.h>
#include "time.h"
#include "config.h"
#include "secret.h"

#ifndef OTA_CHECK_INTERVAL_MS
#define OTA_CHECK_INTERVAL_MS (60UL * 60UL * 1000UL)
#endif
#ifndef FW_VERSION
#define FW_VERSION 0
#endif
#define OTA_TIME_VALID_THRESHOLD 1700000000UL

extern uint16_t otaFailures;

unsigned long otaLastCheck = 0;
bool otaFirstRun = true;

void doOTA()
{
  Serial.printf("OTA: kontrola z %s (verze %d)\n", OtaUrl, (int)FW_VERSION);
  WiFiClientSecure client;
  client.setCACert(OtaRootCA);

  HTTPUpdate updater(30000);
  updater.setAuthorization(OtaUser, OtaPassword);
  updater.onStart([]() { esp_task_wdt_reset(); });
  updater.onEnd([]() { esp_task_wdt_reset(); });
  updater.onProgress([](int cur, int total) { esp_task_wdt_reset(); });
  updater.onError([](int err) { Serial.printf("OTA: chyba %d\n", err); });
  updater.rebootOnUpdate(true);

  t_httpUpdate_return ret = updater.update(client, OtaUrl, String((int)FW_VERSION));

  switch(ret)
  {
    case HTTP_UPDATE_FAILED:
      Serial.printf("OTA: SELHALA (%d): %s\n", updater.getLastError(), updater.getLastErrorString().c_str());
      if(otaFailures < 65535)
      {
        otaFailures++;
      }
    break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("OTA: firmware je aktualni.");
    break;
    case HTTP_UPDATE_OK:
      Serial.println("OTA: OK.");
    break;
  }
}

void otaLoop()
{
  if(!otaFirstRun && millis() - otaLastCheck < OTA_CHECK_INTERVAL_MS)
  {
    return;
  }
  if(WiFi.status() != WL_CONNECTED)
  {
    return;
  }
  if(time(nullptr) < OTA_TIME_VALID_THRESHOLD)
  {
    return;
  }
  if(otaFirstRun)
  {
    Serial.printf("OTA: interval kontroly %lu ms\n", (unsigned long)OTA_CHECK_INTERVAL_MS);
  }
  otaFirstRun = false;
  otaLastCheck = millis();
  doOTA();
}
