#pragma once

#include <Arduino.h>

// Default header
const uint8_t COMTOOL_HEADDER[] = {0xAA, 0xCC, 0xEE, 0xBB};

void ComToolPack(uint8_t *buff, int buff_len,
                 const uint8_t *header, int header_len,
                 const char *name,
                 double x, double y);

template <typename T>
void ComToolPlot(String name, T value)
{
    uint8_t buff[64];

    // use the default header, variable name as name, timestamp as x, value as y
    ComToolPack(buff, sizeof(buff), COMTOOL_HEADDER, sizeof(COMTOOL_HEADDER), name.c_str(), millis(), value);
    Serial.write(buff, sizeof(buff));
}