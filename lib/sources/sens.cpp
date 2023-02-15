#include "sens.h"
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

SoftwareSerial sGPS(GPS_RX, GPS_RX);
TinyGPSPlus gps;

void Sens::init()
{
    sGPS.begin(GPS_BAUD);
    pinMode(ADC_PIN,INPUT);
}

GPSData Sens::getDataLocation()
{
    GPSData gData;
    gData.latitude = 0;
    gData.longitude = 0;
    gData.status = false;
    while (sGPS.available() > 0)
    {
        if (!gps.encode(sGPS.read()))
        {
            continue;
        }
        if (!gps.location.isValid())
        {
            break;
        }
        gData.latitude = gps.location.lat();
        gData.longitude = gps.location.lng();
        gData.status = true;
    }
    return gData;
}

uint16_t Sens::getADCData(){
    return analogRead(ADC_PIN);
}