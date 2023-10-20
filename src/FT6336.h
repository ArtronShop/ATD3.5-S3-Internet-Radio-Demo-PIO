#ifndef __FT6336_H__
#define __FT6336_H__

#include "Arduino.h"
#include <Wire.h>

#define FT6336_ADDR 0x38

class FT6336 {
    public: 
        FT6336() ;

        void init() ;
        uint8_t read(uint16_t *, uint16_t *) ;

};

extern FT6336 touch;

#endif