#ifndef SYSTEMTIMEINTERFACE_H
#define SYSTEMTIMEINTERFACE_H

#include <Arduino.h>

namespace sys_time
{
    unsigned long hal_millis() {
        return millis();
    };
    
    unsigned long hal_micros() {
        return micros();
    };
}

#endif // SYSTEMTIMEINTERFACE_H