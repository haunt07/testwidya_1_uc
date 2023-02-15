#ifndef R_TRANS
#define R_TRANS

#define GSM_TX 17
#define GSM_RX 16
#define GSM_DTR 5
#define GSM_PWR 18
#define GSM_RST 19

#define TINY_GSM_MODEM_SIM800
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 38400

#define TINY_GSM_RX_BUFFER 750
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#define MQTT_BUFF_SIZE 600

#include <Arduino.h>

struct TransSetting
{
    char gprsApn[20];
    char gprsUsername[20];
    char gprsPassword[20];
    char mqttServer[20];
    char mqttID[20];
    char mqttUsername[20];
    char mqttPassword[20];
    uint16_t mqttPort;
};

class Trans
{
public:
    void begin(TransSetting ts, void (*callback)(String &topic, String &payload));
    void setMQTTHost(char *host, uint16_t port);
    bool connect();
    void disconnect();
    void request(String request, uint16_t ms);
    void breakFromWaitRequest();
    void sendDataADC(uint16_t data);
    void sendDataLocation(double latitude,double longitude);
    uint16_t getBatteryVoltage();
    void powerOFF();
    void powerON();
    bool connectNetwork();
    bool connectServer();

private:
    volatile bool flagRequstStatus;
    TransSetting tSetting;
    void genTopic(char *ID, char *topic, char *res);
};

#endif