#include "trans.h"
#include <TinyGsmClient.h>
#include <MQTT.h>

#define SerialGSM Serial2

TinyGsm modem(SerialGSM);
TinyGsmClient client(modem);
MQTTClient mqtt(MQTT_BUFF_SIZE);

char topicRequest[] = "request";
char topicResponse[] = "response";
char topicADC[] = "ADC";
char topicLocation[] = "Location";

void Trans::begin(TransSetting ts, void (*callback)(String &topic, String &payload))
{
    this->tSetting = ts;
    pinMode(GSM_PWR, OUTPUT);
    pinMode(GSM_DTR, OUTPUT);
    pinMode(GSM_RST, OUTPUT);
    digitalWrite(GSM_DTR, HIGH);
    digitalWrite(GSM_PWR, LOW);
    digitalWrite(GSM_RST, HIGH);
    delay(100);
    while (TinyGsmAutoBaud(SerialGSM, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX) == 0)
    {
        digitalWrite(GSM_PWR, HIGH);
        delay(1100);
        digitalWrite(GSM_PWR, LOW);
        delay(3100);
    }
    modem.restart();
    delay(3100);
    mqtt.begin(this->tSetting.mqttServer, this->tSetting.mqttPort, client);
    mqtt.onMessage(callback);
}

void Trans::setMQTTHost(char *host, uint16_t port)
{
    mqtt.setHost(host, port);
}

bool Trans::connect()
{
    char top[40];
    bool simCon = true;
    if (!modem.isNetworkConnected())
    {
        if (!modem.waitForNetwork())
        {
            simCon = false;
        }
    }

    if (!modem.isGprsConnected() && simCon)
    {
        if (!modem.gprsConnect(this->tSetting.gprsApn, this->tSetting.gprsUsername, this->tSetting.gprsPassword))
        {
            simCon = false;
        }
    }

    if (!mqtt.connected() && simCon)
    {
        simCon = false;
        unsigned long tDelay = millis();
        genTopic(this->tSetting.mqttID, topicResponse, top);
        while (millis() - tDelay < 10000)
        {
            mqtt.connect(this->tSetting.mqttID, this->tSetting.mqttUsername, this->tSetting.mqttPassword);
            mqtt.subscribe(top);
            delay(1000);
            if (mqtt.connected())
            {
                simCon = true;
                break;
            }
        }
    }
    return simCon;
}

void Trans::disconnect()
{
    char top[40];
    if (mqtt.connected())
    {
        genTopic(this->tSetting.mqttID, topicResponse, top);
        mqtt.unsubscribe(top);
        mqtt.disconnect();
    }
    if (modem.isGprsConnected())
    {
        modem.gprsDisconnect();
    }
}

void Trans::request(String request, uint16_t ms)
{
    char top[40];
    genTopic(this->tSetting.mqttID, topicRequest, top);
    mqtt.publish(top, request);
    flagRequstStatus = true;
    uint32_t tDelay = millis();
    while (millis() - tDelay < ms && flagRequstStatus)
    {
        mqtt.loop();
    }
}

void Trans::breakFromWaitRequest()
{
    flagRequstStatus = false;
}

void Trans::sendDataADC(uint16_t data)
{
    char top[40];
    String dfRAVG = "{\"dataADC_MAVG\":" + String(data) + "}";
    genTopic(this->tSetting.mqttID, topicADC, top);
    mqtt.publish(top, dfRAVG);
}

void Trans::sendDataLocation(double latitude,double longitude)
{
    char top[40];
    String dfLocation = "{\"latitude\":" + String(latitude) +
                        ",\"longitude\":" + String(longitude) + "}";
    genTopic(this->tSetting.mqttID, topicLocation, top);
    mqtt.publish(top, dfLocation);
}

uint16_t Trans::getBatteryVoltage()
{
    return modem.getBattVoltage();
}

void Trans::genTopic(char *ID, char *topic, char *res)
{
    uint8_t iter1;
    memset(res, 0, 40);
    for (iter1 = 0; iter1 < 20; iter1++)
    {
        if (ID[iter1] != 0)
        {
            res[iter1] = ID[iter1];
        }
        else
        {
            res[iter1] = '/';
            iter1 += 1;
            break;
        }
        char res[40];
    }
    for (uint8_t iter2 = 0; iter2 <= 20; iter2++)
    {
        if (topic[iter2] != 0)
        {
            res[iter1 + iter2] = topic[iter2];
        }
        else
        {
            break;
        }
    }
}

void Trans::powerOFF()
{
    while (modem.testAT(1000))
    {
        digitalWrite(GSM_PWR, HIGH);
        delay(1100);
        digitalWrite(GSM_PWR, LOW);
        delay(3100);
    }
}

void Trans::powerON()
{
    while (!modem.testAT(1000))
    {
        digitalWrite(GSM_PWR, HIGH);
        delay(1100);
        digitalWrite(GSM_PWR, LOW);
        delay(3100);
    }
    modem.init();
}

bool Trans::connectNetwork()
{
    bool simCon = true;
    if (!modem.isNetworkConnected())
    {
        if (!modem.waitForNetwork())
        {
            simCon = false;
        }
    }

    if (!modem.isGprsConnected() && simCon)
    {
        if (!modem.gprsConnect(this->tSetting.gprsApn, this->tSetting.gprsUsername, this->tSetting.gprsPassword))
        {
            simCon = false;
        }
    }
    return simCon;
}
bool Trans::connectServer()
{
    char top[40];
    bool simCon = true;
    if (!mqtt.connected())
    {
        simCon = false;
        unsigned long tDelay = millis();
        genTopic(this->tSetting.mqttID, topicResponse, top);
        while (millis() - tDelay < 10000)
        {
            mqtt.connect(this->tSetting.mqttID, this->tSetting.mqttUsername, this->tSetting.mqttPassword);
            mqtt.subscribe(top);
            delay(1000);
            if (mqtt.connected())
            {
                simCon = true;
                break;
            }
        }
    }
    return simCon;
}