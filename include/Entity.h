#ifndef Entity_h
#define Entity_h

#include "Arduino.h"

class Entity
{
public:
    Entity(unsigned int chan, signed int rssi, String mac);
    unsigned int getCH();
    signed int getRSSI();
    String getMAC();

private:
    unsigned int chan;
    signed int rssi;
    String mac;
};

#endif