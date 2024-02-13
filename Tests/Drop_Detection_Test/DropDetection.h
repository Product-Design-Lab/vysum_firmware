#ifndef DROP_DETECTION_H
#define DROP_DETECTION_H

#include "APDS_Data.h"
#include <Arduino_APDS9960.h>



class APDS_DropSensor
{
public:
    APDS_Data data;

    void init();
    void update();

};

extern APDS_DropSensor drop_sensor;

#endif
