#include <Arduino.h>
#include <sens.h>
#include <mavg.h>

#define ADC_INTERVALMOVINGAVG 100

static Sens sens;
static GPSData dataLocation;
static MAVG movAvg;

static SemaphoreHandle_t mutexDataLocation;
static SemaphoreHandle_t mutexDataADC;

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
      String dfLocation = "{\"latitude\":" + String(gData.latitude) +
                          ",\"longitude\":" + String(gData.longitude) + "}";
      Serial.println(dfLocation);
    }
    else
    {
      Serial.println("Data Location not ready");
    }
    String dfRAVG = "{\"dataADC_MAVG\":" + String(resMovAvg) + "}";
    Serial.println(dfRAVG);
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

void setup()
{
  Serial.begin(115200);
  sens.init();
  dataLocation.status = false;
  movAvg.init(ADC_INTERVALMOVINGAVG);

  mutexDataLocation = xSemaphoreCreateMutex();
  mutexDataADC = xSemaphoreCreateMutex();

  xTaskCreate(gpsTask, "Task GPS", 1024, NULL, 1, NULL);
  xTaskCreate(ADCTask, "Task ADC", 1024, NULL, 1, NULL);
  xTaskCreate(simTask, "Task SIM", 4096, NULL, 1, NULL);
}

uint16_t iter = 0;
void loop()
{
}