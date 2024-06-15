/*
  LOPCLibrary.cpp - Source file for LOPCLibrary.
  
  LOPCLibrary Purpose: 
   *Reads and writes values for Teensy serial number, instrument type, and file number.
   *Creating files that are stored on the SD card
   
  Author: Melissa A Mantey
  Date: 3.29.17
  Version 4
  
*/

#include "LOPCLibrary_revF.h"

//this function creates the library's constructor
LOPCLibrary::LOPCLibrary(int pin)
{
  pinMode(pin, OUTPUT);
  _pin = pin;
  //Serial.begin(115200);
  delay(1000);//while (!Serial); // Wait until Serial is ready
//  Serial.println("SD Card Setup");
//  if(!SD.begin(BUILTIN_SDCARD)){
//   Serial.println("Warning,SD card not inserted");
//  }
}                                              

void LOPCLibrary::SetUp(){
   //DIO Setup
   pinMode(PUMP1_PWR, OUTPUT);
   pinMode(PUMP2_PWR, OUTPUT);
   pinMode(HEATER1, OUTPUT);
   pinMode(HEATER2, OUTPUT);
   pinMode(PHA_POWER, OUTPUT);
   pinMode(PULSE_LED, OUTPUT);
   pinMode(RS41_PWR, OUTPUT);

   pinMode(I_PUMP1, INPUT_DISABLE);
   pinMode(I_PUMP2, INPUT_DISABLE);
         
   delay(1000);
   //LTC2983 Setup
   
   pinMode(CHIP_SELECT, OUTPUT); // Configure chip select pin on Linduino
   pinMode(RESET,OUTPUT);
   pinMode(INTERUPT, INPUT);
   digitalWrite(RESET, HIGH);
   delay(100);
   SPI.begin();
   //SPI.setClockDivider(SPI_CLOCK_DIV128);
    
  analogReadRes(12); // set ADC resolution to this many bits
  analogReadAveraging(32); // average this many readings
  //analogReference(EXTERNAL);//Use external 3.0V reference
  
  //Serial.println("Parameters Configured");
    
   

}

void LOPCLibrary::configure_memory_table() 
{
  uint16_t start_address;
  //uint16_t table_length;
  // int i;

 //Configurations generated using Analog Devices LTC2983 Windows Demo Program

  //Configuration for NTCLE413E2103F102L
  // From: http://www.vishay.com/docs/29078/ntcle413.pdf
  
  uint32_t NTCLE413E2103F102L_steinhart_hart_coefficients[] =
  {
    979823667,  // -- For coefficient 0.0008809000137262046
    964985049,  // -- For coefficient 0.00025273000937886536
    0,  // -- For coefficient 0.0
    877110214,  // -- For coefficient 1.8592899664326978e-07
    0,  // -- For coefficient 0.0
    0   // -- For coefficient 0.0
  };
  start_address = 772;	// Real address = 6*30 + 0x250 = 772
  write_custom_steinhart_hart(CHIP_SELECT, NTCLE413E2103F102L_steinhart_hart_coefficients, start_address);
 
  //Configuration for B57550G1103F00
  // From: https://en.tdk.eu/inf/50/db/ntc_13/NTC_Glass_enc_sensors_G550_G1550.pdf
  // Thermistors used on heated inlet
  
  uint32_t B57550G1103F00_steinhart_hart_coefficients[] =
  {
    979253295,  // -- For coefficient 0.00084769
    965274701,  // -- For coefficient 0.00026116
    0,  // -- For coefficient 0.0
    873131636,  // -- For coefficient 1.2939e-07
    0,  // -- For coefficient 0.0
    0   // -- For coefficient 0.0
  };
  start_address = 712;	// Real address = 6*20 + 0x250 = 712
  write_custom_steinhart_hart(CHIP_SELECT, B57550G1103F00_steinhart_hart_coefficients, start_address);
 	
}


void LOPCLibrary::ConfigureChannels()
{
   //configure_channels(); 
   //uint8_t channel_number;
     uint32_t channel_assignment_data;
   
     // ----- Channel 2: Assign Sense Resistor -----
     channel_assignment_data = 
       SENSOR_TYPE__SENSE_RESISTOR |
       (uint32_t) 0xFA000 << SENSE_RESISTOR_VALUE_LSB;    // sense resistor - value: 1000.
     assign_channel(CHIP_SELECT, 2, channel_assignment_data);
     // ----- Channel 4: Assign Thermistor 44006 10K@25C -----
     channel_assignment_data = 
       SENSOR_TYPE__THERMISTOR_CUSTOM_STEINHART_HART |
       THERMISTOR_RSENSE_CHANNEL__2 |
       THERMISTOR_DIFFERENTIAL |
       THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
       THERMISTOR_EXCITATION_CURRENT__1UA|
       (uint32_t) 0x1E << THERMISTOR_CUSTOM_ADDRESS_LSB; //Custom thermistor at address: 30
     assign_channel(CHIP_SELECT, 4, channel_assignment_data);
     // ----- Channel 6: Assign Thermistor 44006 10K@25C -----
     channel_assignment_data = 
       SENSOR_TYPE__THERMISTOR_CUSTOM_STEINHART_HART |
       THERMISTOR_RSENSE_CHANNEL__2 |
       THERMISTOR_DIFFERENTIAL |
       THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
       THERMISTOR_EXCITATION_CURRENT__1UA|
       (uint32_t) 0x1E << THERMISTOR_CUSTOM_ADDRESS_LSB; //Custom thermistor at address: 30
     assign_channel(CHIP_SELECT, 6, channel_assignment_data);
     // ----- Channel 8: Assign Thermistor 44006 10K@25C -----
     channel_assignment_data = 
       SENSOR_TYPE__THERMISTOR_CUSTOM_STEINHART_HART |
       THERMISTOR_RSENSE_CHANNEL__2 |
       THERMISTOR_DIFFERENTIAL |
       THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
       THERMISTOR_EXCITATION_CURRENT__1UA|
       (uint32_t) 0x1E << THERMISTOR_CUSTOM_ADDRESS_LSB; //Custom thermistor at address: 30
     assign_channel(CHIP_SELECT, 8, channel_assignment_data);
     // ----- Channel 10: Assign Thermistor 44006 10K@25C -----
     channel_assignment_data = 
       SENSOR_TYPE__THERMISTOR_CUSTOM_STEINHART_HART |
       THERMISTOR_RSENSE_CHANNEL__2 |
       THERMISTOR_DIFFERENTIAL |
       THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
       THERMISTOR_EXCITATION_CURRENT__1UA|
       (uint32_t) 0x1E << THERMISTOR_CUSTOM_ADDRESS_LSB; //Custom thermistor at address: 30
     assign_channel(CHIP_SELECT, 10, channel_assignment_data);
     // ----- Channel 12: Assign Thermistor 44006 10K@25C -----
     channel_assignment_data = 
       SENSOR_TYPE__THERMISTOR_44006_10K_25C |
       THERMISTOR_RSENSE_CHANNEL__2 |
       THERMISTOR_DIFFERENTIAL |
       THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
       THERMISTOR_EXCITATION_CURRENT__AUTORANGE;
     assign_channel(CHIP_SELECT, 12, channel_assignment_data);
     // ----- Channel 14: Assign Thermistor 44006 10K@25C -----
     channel_assignment_data = 
       SENSOR_TYPE__THERMISTOR_CUSTOM_STEINHART_HART |
       THERMISTOR_RSENSE_CHANNEL__2 |
       THERMISTOR_DIFFERENTIAL |
       THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
       THERMISTOR_EXCITATION_CURRENT__1UA|
       (uint32_t) 0x1E << THERMISTOR_CUSTOM_ADDRESS_LSB; //Custom thermistor at address: 30
     assign_channel(CHIP_SELECT, 14, channel_assignment_data);
     // ----- Channel 16: Assign Thermistor 44006 10K@25C -----
     channel_assignment_data = 
       SENSOR_TYPE__THERMISTOR_CUSTOM_STEINHART_HART |
       THERMISTOR_RSENSE_CHANNEL__2 |
       THERMISTOR_DIFFERENTIAL |
       THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
       THERMISTOR_EXCITATION_CURRENT__1UA|
       (uint32_t) 0x1E << THERMISTOR_CUSTOM_ADDRESS_LSB; //Custom thermistor at address: 30
     assign_channel(CHIP_SELECT, 16, channel_assignment_data);
     // ----- Channel 18: Assign Sense Resistor -----
     channel_assignment_data = 
       SENSOR_TYPE__SENSE_RESISTOR |
       (uint32_t) 0xFA000 << SENSE_RESISTOR_VALUE_LSB;   // sense resistor - value: 1000.
     assign_channel(CHIP_SELECT, 18, channel_assignment_data);
     // ----- Channel 20: Assign Custom therm -----
     channel_assignment_data = 
       SENSOR_TYPE__THERMISTOR_CUSTOM_STEINHART_HART |
       THERMISTOR_RSENSE_CHANNEL__18 |
       THERMISTOR_DIFFERENTIAL |
       THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
       THERMISTOR_EXCITATION_CURRENT__1UA|
       (uint32_t) 0x1E << THERMISTOR_CUSTOM_ADDRESS_LSB; //Custom thermistor at address: 30
     assign_channel(CHIP_SELECT, 20, channel_assignment_data);
      
   //Serial.println("Channels Configured"); 
    
   //configure_global_parameters();
     // -- Set global parameters
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xF0, TEMP_UNIT__C |
    REJECTION__50_60_HZ);
  // -- Set any extra delay between conversions (in this case, 0*100us)
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xFF, 0);
}


float LOPCLibrary::MeasureLTC2983(int channel){
   float temp;
   temp =   measure_channel(CHIP_SELECT, channel, TEMPERATURE);      // Ch 4: Thermistor 44006 10K@25C
   return temp;
} 

void LOPCLibrary::SleepLTC2983(void){
	transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0x000, 0x97); //put the LTC2983 to sleep per page 58 of spec sheet
	}
	
//This function reads the instrument instrument type and returns it as an int. Returns -1 if unsuccessful
int LOPCLibrary::InstrumentType(){//Function begin  
  //Get what is currently written to EEPROM (Type)     
  Serial.print("Current Type is: ");
  char val = EEPROM.read(0);
  Serial.println((int)val);
  return val;
  /*
  //Ask user to input data  
  Serial.println("Enter instrument type( 0 - OPC, 1 -CN, 2 - PHA, 3- Other): ");
  //while(Serial.available() == 0){}
  delay(8000);
  int type = Serial.parseInt();
  Serial.println(type);
  
  //Check to make sure there was no user error
  if((type >= 0) && (type < 4)){ //begin if statement. 
     EEPROM.write(0, (char)type); //If input was correct, write new value to memory

  delay(100);   //Wait so that data can be stored.
  
  int value = EEPROM.read(0); //Store data as an int
  
  //Print stored value
  Serial.print("Stored value is:  ");
  Serial.println(value);
  Serial.println();
  
  return value;
  }//end if statement
  else
  {//begin else statement
    int value = EEPROM.read(0);
    Serial.println("Invalid entry! Type not updated");
    Serial.print("Stored value is:  ");
    Serial.println(value);
    Serial.println();

    return -1;
  }//End else statement
  */
}//Function end

//reads the instrument serial number from flash and returns it as an int.
int LOPCLibrary::SerialNumber()           
{
  //Get what is currently written to EEPROM (Serial Number)
  Serial.print("Current Serial Number is: ");
  char val = EEPROM.read(1);
  Serial.println((int)val);
  return val;
  /*
  //Ask for user to input serial number
  Serial.println("Enter instrument serial number (0-255): ");
   //delay(2000); //Wait for response
   //while(Serial.available() == 0){}
     delay(8000);
   int type = Serial.parseInt();

  if((type>=0) && (type<255)){
     EEPROM.write(1, (char)type);

    delay(100);

  int value = EEPROM.read(1);

  Serial.print("Stored value is:  ");
  Serial.println(value);
  Serial.println();
  
  return value;
  }
  else
  {
    Serial.println("Invalid entry! Serial number not updated");
    Serial.println();
    return -1;
  }
*/
}

int LOPCLibrary::FileNumber()             //reads the file number (16 bit int) from EEPROM
{
  //Gets what is currently written to EEPROM (File Counter)
  Serial.print("Current file counter is: ");
  char Hi_val = EEPROM.read(2);
  char Lo_val = EEPROM.read(3);
  uint16_t int_val = Hi_val*256 + Lo_val; //recombine bytes into 16 bit int
  Serial.println((int)int_val);
  return int_val;
  /*
  //
  bool UserEnteredInput = true;
  Serial.println("Enter file counter (0-65535): ");
   
   int type;
   type = Serial.parseInt();
         
  if((type>=0) && (type<65535)){
    uint16_t TwoBytes = (unsigned int)type;  //recast type as an unsigned int
    uint8_t LowByte = TwoBytes & 0xFF; //get the low byte
    uint8_t HiByte = TwoBytes >> 8; //get the high byte
    
     EEPROM.write(2, (char)HiByte);  //Write high byte first
     delay(100);  //EEPROM writes take a while
     EEPROM.write(3, (char)LowByte);  //Write low byte

    delay(100);
   
  char Hi_val = EEPROM.read(2);
  char Lo_val = EEPROM.read(3);
  uint16_t val = Hi_val*256 + Lo_val; //recombine bytes into 16 bit int
  
  
  
  Serial.print("Stored value is:  ");
  Serial.println((int)type);
  int value = (int)type;
  Serial.println();
  
  return (int)type;
  }
  else
  {
    Serial.println("Invalid entry! File counter not updated");
    Serial.println();
    return int_val;
  }
  */
}

int LOPCLibrary::IncrementFile() //increases the file number by one each time a file is written.
{
  //Get current file counter.
  char Hi_val = EEPROM.read(2);
  char Lo_val = EEPROM.read(3);
  uint16_t int_val = Hi_val*256 + Lo_val; //recombine bytes into 16 bit int
  
  //Get incremented file counter.
  uint16_t inc_int_val = int_val + 1; //add one to 16 bit int
  
  uint16_t TwoBytes = (unsigned int)inc_int_val;  //recast type as an unsigned int
    uint8_t LowByte = TwoBytes & 0xFF; //get the low byte
    uint8_t HiByte = TwoBytes >> 8; //get the high byte
    
     EEPROM.write(2, (char)HiByte);  //Write high byte first
     delay(100);  //EEPROM writes take a while
     EEPROM.write(3, (char)LowByte);  //Write low byte
     
  _filecount++;
  
  return inc_int_val;
}

String LOPCLibrary::CreateFileName()
{
  Serial.println("File name being created. . .");
  String filename;
  
  //Get file number from EEPROM to use as an extension
  int type = EEPROM.read(0);
  int serial = EEPROM.read(1);
  char Hi_val = EEPROM.read(2);
  char Lo_val = EEPROM.read(3);
  uint16_t filenum = Hi_val*256 + Lo_val;
  
  //First two letters of file name are generated
  if(type == 0)
    filename = "OP";
  else if(type == 1)
    filename = "CN";
  else if(type == 2)
    filename = "PH";
  else if(type == 3)
    filename = "TS";
  else
    filename = String(type);
  
  //Next two letters (xx) of file name are generated.
  if(serial >= 0 && serial <= 9)
    filename = filename + "0" + String(serial);
  else
    filename = filename + String(serial);

  //Next four letters (yyyy)
  if(filenum >= 0 && filenum <= 9)
    filename = filename + "000" + String(filenum);
  else if(filenum > 9 && filenum <= 99)
    filename = filename + "00" + String(filenum);
  else if(filenum > 99 && filenum <= 999)
    filename = filename + "0" + String(filenum);
  else
    filename = filename + String(filenum);

  //.txt ending added
  filename = filename + ".txt";
  
  Serial.println(filename);
  Serial.println();

  IncrementFile();
  return filename;
}

bool LOPCLibrary::FileExists(String FileName)
{
   char filename[100];
     FileName.toCharArray(filename, 100);
   if(SD.exists(filename))
   {
      Serial.println("File name already exists. Returning true");
      Serial.println();
      return true;
   }
   Serial.println("File name does not already exist. Returning false");
   Serial.println();
   return false;
}

String LOPCLibrary::GetNewFileName()
{
  Serial.println("New file name being created. . .");
  String filename;
  
  //Get file number from EEPROM to use as an extension
  int type = EEPROM.read(0);
  int serial = EEPROM.read(1);
  char Hi_val = EEPROM.read(2);
  char Lo_val = EEPROM.read(3);
  uint16_t filenum = Hi_val*256 + Lo_val;
  
  //First two letters of file name are generated
  if(type == 0)
    filename = "OP";
  else if(type == 1)
    filename = "CN";
  else if(type == 2)
    filename = "PH";
  else if(type == 3)
    filename = "TS";
  else
    filename = String(type);
  
  //Next two letters (xx) of file name are generated.
  if(serial >= 0 && serial <= 9)
    filename = filename + "0" + String(serial);
  else
    filename = filename + String(serial);

  //Next four letters (yyyy)
  if(filenum >= 0 && filenum <= 9)
    filename = filename + "000" + String(filenum);
  else if(filenum > 9 && filenum <= 99)
    filename = filename + "00" + String(filenum);
  else if(filenum > 99 && filenum <= 999)
    filename = filename + "0" + String(filenum);
  else
    filename = filename + String(filenum);

  //.txt ending added
  filename = filename + ".txt";
  
  Serial.println(filename);
  Serial.println();
  
  IncrementFile();
  
  return filename;
}


bool LOPCLibrary::WriteData(String FileName, String Data)
{
File myFile;

//Serial.print("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    return false;
  }
  //Serial.println("initialization done.");

  // open a new file, write to file and immediately close it:
  //Serial.println("Writing to File: " + FileName);
  char filename[100];
     FileName.toCharArray(filename, 100);
     
  myFile = SD.open(filename, FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    //Serial.print("Writing to file...");
    myFile.print(Data + " ");
	// close the file:
    myFile.close();
    //Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
    return false;
  }
  return true;
}

