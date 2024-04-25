//WattmeterINA219Driver.cpp
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_INA219.h"
#include "WattmeterINA219Driver.h" 

/**
 * @fn DFRobot_INA219_IIC
 * @brief pWire I2C controller pointer
 * @param i2caddr  I2C address
 * @n INA219_I2C_ADDRESS1  0x40   A0 = 0  A1 = 0
 * @n INA219_I2C_ADDRESS2  0x41   A0 = 1  A1 = 0
 * @n INA219_I2C_ADDRESS3  0x44   A0 = 0  A1 = 1
 * @n INA219_I2C_ADDRESS4  0x45   A0 = 1  A1 = 1	 
  */
//DFRobot_INA219_IIC* ina219[8];
DFRobot_INA219_IIC* ina219[4];

// Revise the following two paramters according to actual reading of the INA219 and the multimeter
// for linearly calibration
float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;

const int8_t wn0 = 0;
//const int8_t wn1 = 1;

const int8_t addr1 = 40;
const int8_t addr2 = 41;
const int8_t addr3 = 44;
const int8_t addr4 = 45;

//int8_t init_f[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//extern "C" void wINA219Driver_Init(int8_t id, int8_t wire_number, int8_t slave_address)
int8_t init_f[] = {0, 0, 0, 0};
extern "C" void wINA219Driver_Init(int8_t id, int8_t slave_address)
{ 
    // Habría que tener en cuenta si un objecto esta ya iniciado o no
    // Pero eso es una situación posible #TODO

    /* Initialize the sensor and store the status in init_f */
    //if(wire_number == wn0){
    if(slave_address == addr1) ina219[id] = new DFRobot_INA219_IIC(&Wire, INA219_I2C_ADDRESS1);
    if(slave_address == addr2) ina219[id] = new DFRobot_INA219_IIC(&Wire, INA219_I2C_ADDRESS2);
    if(slave_address == addr3) ina219[id] = new DFRobot_INA219_IIC(&Wire, INA219_I2C_ADDRESS3);
    if(slave_address == addr4) ina219[id] = new DFRobot_INA219_IIC(&Wire, INA219_I2C_ADDRESS4);
    //}

    /*
    if(wire_number == wn1){
        if(slave_address == addr1) ina219[id] = new DFRobot_INA219_IIC(&Wire1, INA219_I2C_ADDRESS1);
        if(slave_address == addr2) ina219[id] = new DFRobot_INA219_IIC(&Wire1, INA219_I2C_ADDRESS2);
        if(slave_address == addr3) ina219[id] = new DFRobot_INA219_IIC(&Wire1, INA219_I2C_ADDRESS3);
        if(slave_address == addr4) ina219[id] = new DFRobot_INA219_IIC(&Wire1, INA219_I2C_ADDRESS4);
    }
    */

    if(ina219[id]){
        if (ina219[id]->begin() == true) init_f[id] = 1; // Device found
        // ELSE: Error, device not found
    }

	//Set BRNG (Bus Voltage Range)
    //ina219->setBRNG(ina219->eIna219BusVolRange_32V);
	
    //Set PGA parameter(Shunt Voltage Only)
    //ina219->setPGA(ina219->eIna219PGABits_1);
    
    //Set BADC parameter (Bus ADC Resolution/Averaging)
	//ina219->setBADC(ina219->eIna219AdcBits_12, ina219->eIna219AdcSample_8);
    
    //Set SADC parameter (Shunt ADC Resolution/Averaging)
	//ina219->setSADC(ina219->eIna219AdcBits_12, ina219->eIna219AdcSample_8);
	
    //Set operation Mode(Bus Voltage Range)
    //ina219->setMode(ina219->eIna219SAndBVolCon);
    
    //Linear calibration
    //ina219->linearCalibrate(/*The measured current before calibration*/ina219Reading_mA, /*The current measured by other current testers*/extMeterReading_mA);
} 
extern "C" void wINA219Driver_Step(int8_t id, int32_t* mA, int32_t* mV)
{ 
    /* If sensor is initialized properly, then read the sensor else return 0 */
    if(init_f[id])              
    { 
        // Obtain sensor power data
        *mA = (int32_t) ina219[id]->getCurrent_mA();
        *mV = (int32_t) (ina219[id]->getBusVoltage_V() * 1000.0f);
    } 
    else
    { 
        *mA = -1; 
        *mV = -1;
    } 
} 
extern "C" void wINA219Driver_Terminate() 
{ 
} 