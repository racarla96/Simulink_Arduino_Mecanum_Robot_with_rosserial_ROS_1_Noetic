//AFMSV2Driver.cpp
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "AFMSV2Driver.h" 
#include "Adafruit_MotorShield.h"   

/*Create object of the motor driver*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

uint8_t init_f_motor;
extern "C" void AFMSV2Driver_Init(void)
{ 
    // Initializing the device driver
    uint16_t freq = 1600;
    Wire1.begin();
    if (!AFMS.begin(freq, &Wire1)) init_f_motor = 0; // Error, device not found
    else init_f_motor = 1; // Device found
} 
extern "C" void AFMSV2Driver_Step(int16_t U1, int16_t U2, int16_t U3, int16_t U4) 
{ 
    if(init_f_motor) {             /* If device is initialized properly, else return 0 */
        if (U1 > 0) M1->run(BACKWARD);
        else if (U1 < 0) {
            M1->run(FORWARD);
            U1 = fabs(U1);
        } else M1->run(RELEASE);
        if (U1 > 4095) U1 = 4095;

        if (U2 > 0) M2->run(FORWARD);
        else if (U2 < 0) {
            M2->run(BACKWARD);
            U2 = fabs(U2);
        } else M2->run(RELEASE);
        if (U2 > 4095) U2 = 4095;

        if (U3 > 0) M3->run(FORWARD);
        else if (U3 < 0) {
            M3->run(BACKWARD);
            U3 = fabs(U3);
        } else M3->run(RELEASE);
        if (U3 > 4095) U3 = 4095;

        if (U4 > 0) M4->run(FORWARD);
        else if (U4 < 0) {
            M4->run(BACKWARD);
            U4 = fabs(U4);
        } else M4->run(RELEASE);
        if (U4 > 4095) U4 = 4095;

        M1->setSpeedFine((uint16_t) U1); // 0 (full off) to 4095 (full on)
        M2->setSpeedFine((uint16_t) U2); // 0 (full off) to 4095 (full on)
        M3->setSpeedFine((uint16_t) U3); // 0 (full off) to 4095 (full on)
        M4->setSpeedFine((uint16_t) U4); // 0 (full off) to 4095 (full on)
    } 
} 
extern "C" void AFMSV2Driver_Terminate() 
{ 
} 