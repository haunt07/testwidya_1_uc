#include "mavg.h"
#include <movingAvg.h>

movingAvg *mavg;

void MAVG::init(uint16_t interval)
{
    mavg = new movingAvg(interval);
    mavg->begin();
}

void MAVG::pushData(uint16_t data)
{
    mavg->reading(data);
}

uint16_t MAVG::getAvg()
{
    return mavg->getAvg();
}