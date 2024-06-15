/*
  LOPCLibrary_revF
  
  Library for . . . 
  *reading/writing important values to flash memory.
  *Creating files that are stored on the SD card
  
  Created by Melissa Mantey
  1.24.17
  Version 5

  11/23 Updated the references to the SD library
  2/24 Update definitions for Mainbaord rev F to support alternative pin declerations for Teensy 4.1
*/

#ifndef OPCLibrary6_h
#define OPCLibrary6_h

#include "Arduino.h"
#include "EEPROM.h"

#include <SD.h>
//#include <SdFatConfig.h>

//Teensy 3.6 specific SD card config
//#define USE_SDIO 1

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <SPI.h>
#include "Wire.h"
#include "stdio.h"
#include "math.h"
//#include <i2c_t3.h>  //specialized Teensy 3.6 version of arduino i2c library

// LTC2983 Temperature IC Libraries
#include "LTC2983_configuration_constants.h"
#include "LTC2983_support_functions.h"
#include "LTC2983_table_coeffs.h"

// GPS Library
#include <TinyGPS++.h>

// Serial Port constants
#define OPCSERIAL Serial1
#define GPSSERIAL Serial4
#define IMET_SERIAL Serial2
#define CNC_SERIAL Serial3

//LTC2983 Constants
#define CHIP_SELECT 10
#define RESET 9
#define INTERUPT 27

//Flow Meter Constants
#define MFS_POWER 24

//DIO Constants
#define PUMP1_PWR 5
#define PUMP2_PWR 6
#define PHA_POWER 4
#define HEATER1 36
#define HEATER2 2
#define PULSE_LED 37
#define RS41_PWR 32

//Analog Constants
#define PUMP1_BEMF A17
#define PUMP2_BEMF A9
#define I_PUMP1 A11
#define I_PUMP2 A12
#define BATTERY_V A16
#define PHA_12V_V A15
#define PHA_3V3_V A8
#define PHA_I A14
#define HEATER1_I A10
#define HEATER2_I A6
#define TEENSY_3V3 A7

//Temperature Channels
#define PUMP1_THERM 4
#define PUMP2_THERM 6
#define HEATER1_THERM 8
#define HEATER2_THERM 10
#define BOARD_THERM 12
#define SPARE_THERM 14
#define PHA_THERM 16
#define OAT_THERM 20


class LOPCLibrary
{
  public:
    LOPCLibrary(int pin);
    void SetUp();//Configures Teensy to LTC2983
    void ConfigureChannels(); //Configure LTC2983 Channel settings
    void configure_memory_table(); // Configure custom thermistor parameters
    void SleepLTC2983(); //put the LTC2983 to sleep
    float MeasureLTC2983(int channel);//returns the temperature of a given channel in degrees C.
    void printGPS();
    void printTemps();
    int InstrumentType(); //reads/writes the instrument type (1,2 or 3) from EEPROM, error checking as above.
    int SerialNumber(); //reads/writes the instrument serial number from flash and returns it as an int. Returns -1 if error occured
    int FileNumber(); //reads/writes the file number from EEPROM, error checking as above. Returns -1 if error occured
    int IncrementFile(); //increments the file number
    int ErrorCheck(int serial, int type, int file); //checks to see if errors occured. Returns # of errors found.   
    String CreateFileName();
    bool FileExists(String FileName); //Check to make sure the file name doesn't already exist, return False if it doesn't exist, true exist.
    String GetNewFileName(); //would create an alternative filename 'OPxxyyyy.1' using the first available extension.
    bool WriteData(String FileName, String Data); //Open the file, write the data, close the file, return true on success.
    float ReadAnalog(int channel);
    
  private:
    int _pin;
    int _filecount;
    //Sd2Card _SD;

};

#endif
