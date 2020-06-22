#ifndef State_h
#define State_h

#include "Arduino.h"

class State
{
public:
    bool check(uint16_t flags);
    void set(uint16_t flags);
    void clear(uint16_t flags);
    uint16_t m_state = 0;
};

#endif