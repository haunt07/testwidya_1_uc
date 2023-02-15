#include <Arduino.h>
#include <sens.h>
#include <mavg.h>
#include <trans.h>

#define ADC_INTERVALMOVINGAVG 100

static Sens sens;
static GPSData dataLocation;
static MAVG movAvg;
static Trans trans;
TransSetting tSetting;

String gprsAPN = "Your GPRS APN";
String gprsUsername = "Your GPRS Username";
String gprsPassword = "Your GPRS Password";
String mqttServer = "Your MQTT HOST";
String mqttID = "Your MQTT ID";
String mqttUsername = "Your MQTT Username";
String mqttPassword = "Your MQTT Password";
uint16_t mqttPort = 1883;

static SemaphoreHandle_t mutexDataLocation;
static SemaphoreHandle_t mutexDataADC;

void transCallback(String &topic, String &payload);
static void gpsTask(void *params);
static void simTask(void *params);
static void ADCTask(void *params);

void setup()
{
  Serial.begin(115200);
  sens.init();
  dataLocation.status = false;
  movAvg.init(ADC_INTERVALMOVINGAVG);

  gprsAPN.toCharArray(tSetting.gprsApn, 20);
  gprsUsername.toCharArray(tSetting.gprsUsername, 20);
  gprsPassword.toCharArray(tSetting.gprsPassword, 20);
  mqttServer.toCharArray(tSetting.mqttServer, 20);
  mqttID.toCharArray(tSetting.mqttID, 20);
  mqttUsername.toCharArray(tSetting.mqttUsername, 20);
  mqttPassword.toCharArray(tSetting.mqttPassword, 20);
  tSetting.mqttPort = mqttPort;

  trans.begin(tSetting, transCallback);

  mutexDataLocation = xSemaphoreCreateMutex();
  mutexDataADC = xSemaphoreCreateMutex();

  xTaskCreate(gpsTask, "Task GPS", 1024, NULL, 1, NULL);
  xTaskCreate(ADCTask, "Task ADC", 1024, NULL, 1, NULL);
  xTaskCreate(simTask, "Task SIM", 40960, NULL, 1, NULL);
}

uint16_t iter = 0;
void loop()
{
}

static void gpsTask(void *params)
{
  while (1)
  {
    GPSData gData = sens.getDataLocation();
    if (gData.status)
    {
      if (xSemaphoreTake(mutexDataLocation, 1000 / portTICK_PERIOD_MS) == pdTRUE)
      {
        dataLocation = gData;
        xSemaphoreGive(mutexDataLocation);
      }
    }
    vTaskDelay(1);
  }
}

static void simTask(void *params)
{
  while (1)
  {
    GPSData gData;
    uint16_t resMovAvg;

    if (!trans.connect())
    {
      vTaskDelay(1);
      continue;
    }

    if (xSemaphoreTake(mutexDataLocation, 1000 / portTICK_PERIOD_MS) == pdTRUE)
    {
      gData = dataLocation;
      xSemaphoreGive(mutexDataLocation);
    }
    else
    {
      vTaskDelay(1);
      continue;
    }

    if (xSemaphoreTake(mutexDataADC, 1000 / portTICK_PERIOD_MS) == pdTRUE)
    {
      resMovAvg = movAvg.getAvg();
      xSemaphoreGive(mutexDataADC);
    }
    else
    {
      vTaskDelay(1);
      continue;
    }

    if (gData.status)
    {
      trans.sendDataLocation(gData.latitude, gData.longitude);
    }
    else
    {
      Serial.println("Data Location not ready");
    }

    trans.sendDataADC(resMovAvg);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

static void ADCTask(void *params)
{
  while (1)
  {
    if (xSemaphoreTake(mutexDataADC, 1000 / portTICK_PERIOD_MS) == pdTRUE)
    {
      uint16_t dataADC = sens.getADCData();
      movAvg.pushData(dataADC);
      xSemaphoreGive(mutexDataADC);
    }
    else
    {
      vTaskDelay(1);
      continue;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
void transCallback(String &topic, String &payload) {}