#ifndef R_SENS
#define R_SENS

#define GPS_RX 15
#define GPS_TX 4
#define GPS_BAUD 9600

#define ADC_PIN 32

#include <Arduino.h>

struct GPSData
{
    double latitude;
    double longitude;
    bool status;
};

class Sens
{
public:
    void init();
    GPSData getDataLocation();
    uint16_t getADCData();
};

#endif