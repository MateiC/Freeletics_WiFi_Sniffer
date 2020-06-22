#include <Entity.h>
#include "Arduino.h"

Entity::Entity(unsigned int chan, signed int rssi, String mac)
{
    Entity::chan = chan;
    Entity::rssi = rssi;
    Entity::mac = mac;
}

unsigned int Entity::getCH()
{
    return chan;
}

signed int Entity::getRSSI()
{
    return rssi;
}

String Entity::getMAC()
{
    return mac;
}