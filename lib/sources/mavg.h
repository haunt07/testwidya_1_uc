#ifndef R_MAVG
#define R_MAVG

#include <Arduino.h>

class MAVG
{
public:
    void init(uint16_t interval);
    void pushData(uint16_t data);
    uint16_t getAvg();
};

#endif