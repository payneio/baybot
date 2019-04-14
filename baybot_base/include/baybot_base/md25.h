#pragma once

#include <stdint.h>

namespace md25 {

    class MD25 {
        
        public:
        MD25();
        ~MD25();

        void LeftMotor(uint8_t);
        void RightMotor(uint8_t);
        double BatteryVoltage();
        double Temperature();
        void LeftEncoder();
        void RightEncoder();
        double Pressure();
        double Altitude();

    };


}