
#include "src/LOPCLibrary_revF.h"
#include <SPI.h>

#define INTERACTIVE
String Version = "PHA_V5_1";

FASTRUN void Get_Baseline(int NumSamples);
FASTRUN int Get_Pulse_fast(unsigned int trigger_level);
void parseCommand(String commandToParse);
void setDACReference(int ref);
void setDACGain(byte Gain);
void setOutputA(unsigned int val);
void setOutputB(unsigned int val);

/* 
 *  New Version for PHA Rev E
 *  New pin mappings, digital threshold detection
 *  and DAC based offset on both gain stages. 
 *  V4.1 : Add interactive mode for setting gains and offsets
 *         Send a 256 bin array to the Main board for each of LG and HG.
 *  V5.0 : New version for T4.1 / PCB RevE that:
 *          - Uses an external DAC
 *          - Shares GPIO6 between both external ADCs
 *. V5.1 : Minor revision to increase the amount of house keeping data saved to the SD card to help diagnose issues with LPC in Strateole 2
 */


/****** These are parameters you may want to adjust *****/
#define CYCLE_TIME 2000  //Pulse count integration time in ms
#define SATURATION 20000 //Maximum real world number of pulses - if this number
                        // is exceeded, we assume pulse threshold is too low.
//#define INIT_THRESHOLD 938  //This is the initial pulse dection threshold out of 1023 with a 3.3V reference -  around 900 works
#define INIT_THRESHOLD 900  // increase this number until the threshold hits the noise floor                
#define HIGH_GAIN_OFFSET 3200 //This sets the 'zero level' near 3.5V for the high gain channel
#define LOW_GAIN_OFFSET 3390  //This sets the 'zero level' near 3.45V for the low gain channel
#define PULSE_WINDOW 100 //the number of ADC samples to collect after a trigger

//EEPROM Addresses
#define EEPROM_FLAG  30 //value to say there is valid pump data in eeprom
#define EEPROM_LG_OFFSET 40 //lg offset address
#define EEPROM_HG_OFFSET 50 //hg offset address
#define EEPROM_THRESHOLD 60 //threshold address

#define SAMPLES 5000

//Setup 10 contiguous digital pins in prt GPIO6 to be shared between ADCs
#define D0 19
#define D1 18
#define D2 14
#define D3 15
#define D4 40
#define D5 41
#define D6 17
#define D7 16
#define D8 22
#define D9 23 

//  Define ADC control pins
#define LG_ADC_OE 2
#define LG_ADC_CLK 3
#define LG_ADC_PD 4

#define HG_ADC_OE 5
#define HG_ADC_CLK 6
#define HG_ADC_PD 7
#define TRIGGERED 29

#define LASER_I_MON A11
#define TRIGGER_PULSE 26
#define DET_ENABLE 20     

#define DEBUG_SERIAL Serial
#define OUTPUT_SERIAL Serial1

// External DAC interface
#define DAC_CS 10
SPISettings DACsettings(50000000, MSBFIRST, SPI_MODE0);

/*** Variables ***/
volatile uint16_t HG_buff[SAMPLES];
volatile uint16_t LG_buff[SAMPLES];
volatile int cnt = 0;
volatile uint16_t dummy;
uint16_t HG_Array[1024];
uint16_t LG_Array[1024];
uint16_t HG_Small_Array[256];
uint16_t LG_Small_Array[256];
char HG_Out_Buffer[4096];
char LG_Out_Buffer[4096];
char HG_Out_Small_Buffer[2048];
char LG_Out_Small_Buffer[2048];
int LG_Buffer_Index = 0;
int HG_Buffer_Index = 0;
int LG_Small_Buffer_Index = 0;
int HG_Small_Buffer_Index = 0;
long pulse_count = 0;
long lg_bad_pulse = 0;
long hg_bad_pulse = 0;
long startTime = 0;
long endTime = 0;
long HG_Offset = 0;
long LG_Offset = 0;
int mode_HG = 0;
int mode_LG = 0;
int mode_HG_index = 0;
int mode_LG_index = 0;
int FrameCounter = 0;
float LaserI = 0.0;
String file;
String TimeStamp;
String OutputString = "";
uint32_t HG_Baseline = 0;
uint32_t LG_Baseline = 0;
int HG_Offset_val = HIGH_GAIN_OFFSET;
int LG_Offset_val = LOW_GAIN_OFFSET;
int Threshold = INIT_THRESHOLD;

String Input_Buff = "";
char C;

LOPCLibrary PHA(13);  //Creates an instance of the EEPROMLibrary


void setup() {

DEBUG_SERIAL.begin(115200); //USB serial
delay(2000);
DEBUG_SERIAL.println(Version);

Serial.print("PHA build ");
Serial.print(__DATE__);
Serial.print(" ");
Serial.println(__TIME__);

/* Set GPIO6 pins to inputs */
pinMode(D0, INPUT);
pinMode(D1, INPUT);
pinMode(D2, INPUT);
pinMode(D3, INPUT);
pinMode(D4, INPUT);
pinMode(D5, INPUT);
pinMode(D6, INPUT);
pinMode(D7, INPUT);
pinMode(D8, INPUT);
pinMode(D9, INPUT);

pinMode(TRIGGER_PULSE, OUTPUT);   // input from threshold comparator (H=Pulse Dectected)
pinMode(TRIGGERED,OUTPUT); //digital pin to pull high when triggered

/* Set Control pins to outputs */
pinMode(LG_ADC_PD, OUTPUT);
pinMode(LG_ADC_OE, OUTPUT);
pinMode(HG_ADC_PD, OUTPUT);
pinMode(HG_ADC_OE, OUTPUT);
pinMode(DET_ENABLE, OUTPUT);
pinMode(HG_ADC_CLK, OUTPUT);
pinMode(LG_ADC_CLK, OUTPUT);

/* Enable ADCS */
digitalWrite(LG_ADC_PD, LOW);
digitalWrite(HG_ADC_PD, LOW);
digitalWrite(HG_ADC_OE, HIGH);  //disable outputs (active on OE low)
digitalWrite(LG_ADC_OE, HIGH);  //disable outputs (active on OE low)

digitalWrite(DET_ENABLE, LOW);
digitalWrite(HG_ADC_CLK, HIGH);        // Set ADC clock lines high, ADC converts on falling edge
digitalWrite(LG_ADC_CLK, HIGH);        // Set ADC clock lines high, ADC converts on falling edge

//DAC SPI Config
pinMode(DAC_CS, OUTPUT);
SPI.begin();
SPI.beginTransaction(DACsettings);

/*Check to see if we have valid offset set points in EEPROM and read if we do */

  if (EEPROM.read(EEPROM_FLAG) == 0x01)
  {
    EEPROM.get(EEPROM_HG_OFFSET, HG_Offset_val);
    DEBUG_SERIAL.print("Saved HG Baseline Offset Setting loaded: ");
    DEBUG_SERIAL.println(HG_Offset_val);
    EEPROM.get(EEPROM_LG_OFFSET, LG_Offset_val );
    DEBUG_SERIAL.print("Saved LG Baseline Offset Setting loaded: ");
    DEBUG_SERIAL.println(LG_Offset_val);
    EEPROM.get(EEPROM_THRESHOLD, Threshold );
    DEBUG_SERIAL.print("Saved Threshold Setting loaded: ");
    DEBUG_SERIAL.println(Threshold);
    
  }
  else
  {
    DEBUG_SERIAL.println("Using Default PHA settings");
  }


/*Set the pulse baseline offset */



setDACReference(1);
setDACGain(0);
setOutputA(LG_Offset_val);      // 12 bit DAC output 0V to 1.2V, (3600/4096)*1.2V = 1.055V
setOutputB(HG_Offset_val);

/* Zero buffers */
memset(HG_Array,0,sizeof(HG_Array));
memset(LG_Array,0,sizeof(LG_Array));

OUTPUT_SERIAL.begin(500000); //Hardware sertial to mainboard


digitalWrite(DET_ENABLE, HIGH); //Enable the optical head
delay(100);

  /*Read Instrument type/serial from EEPROM and create the file on the SD Card */
  
  int type = PHA.InstrumentType();
  int serial = PHA.SerialNumber();
  int filenum = PHA.FileNumber();
  DEBUG_SERIAL.println("Startup");
  DEBUG_SERIAL.print("Type: ");
  DEBUG_SERIAL.println(type);
  DEBUG_SERIAL.print("Serial: ");
  DEBUG_SERIAL.println(serial);
  DEBUG_SERIAL.print("File Number: ");
  DEBUG_SERIAL.println(filenum);

  //This creates a file name that can be stored on the SD card
  file = PHA.CreateFileName();
  DEBUG_SERIAL.print("Filename: ");
  DEBUG_SERIAL.println(file);   

  //This checks to see if the file name has been used.
  if (PHA.FileExists(file) == true)
    file = PHA.GetNewFileName();

  String header = "Elapsed Time [ms], Laser Monitor [V], Threshold [#/4095], Low Gain Channels, High Gain Channels \n";
      PHA.WriteData(file, header); 

  /* Setup the Analog input channels */  
  analogReadAveraging(32);

startTime = millis();

}

void loop() 
{
 int i;
 int j;
 int min_hg = 1023;
 int min_lg = 1023;
 int mindex_lg = 0;
 int mindex_hg = 0;


if (Get_Pulse_fast(Threshold)); //wait for then digitize the next pulse
 {
  
    for(i = PULSE_WINDOW; i >0 ; i--) //Find minimum of high gain sample array
    {
    if(HG_buff[i] < min_hg)
     {
       min_hg = HG_buff[i];
       mindex_hg = i;
      }
    
    if(LG_buff[i] < min_lg) //Find minimum of low gain sample array
     {
       min_lg = LG_buff[i];
       mindex_lg = i;
      }
    }
    
    
    pulse_count++; 
    HG_Array[min_hg] = HG_Array[min_hg] + 1; //increment appropriate bin in high gain pulse spectrum array
    LG_Array[min_lg] = LG_Array[min_lg] + 1; //increment appropriate bin in low gain pulse spectrum array
    
    if(mindex_lg > PULSE_WINDOW-10) 
    {
      lg_bad_pulse++; //Count "bad pulses" in low gain path
    }
     if(mindex_hg > PULSE_WINDOW -10) 
    {
      hg_bad_pulse++; //Count "bad pulses" in high gain path
    }
 }  

/* After CYCLE_TIME) seconds of pulse aquisition, save and output the pulse spectrum */

if((millis() - startTime )>= CYCLE_TIME)  
{
  long timeStart = micros();
  DEBUG_SERIAL.println("Processing Pulses");
  mode_HG = 0;
  mode_LG = 0;
  mode_HG_index = 0;
  mode_LG_index = 0;
  LG_Buffer_Index = 0;
  LG_Small_Buffer_Index = 0;
  HG_Buffer_Index = 0;  
  HG_Small_Buffer_Index = 0;

  
  /* Create a 256 element down sampled array to output */

  for (i = 0; i < 255; i++)
  {
    for(j = 0; j < 4; j++)
    {
    LG_Small_Array[i] = LG_Small_Array[i] + LG_Array[i*4+j];
    HG_Small_Array[i] = HG_Small_Array[i] + HG_Array[i*4+j];
    }
  }

  /*Find the mode of the HG & LG arrays, primarily for calibration */
  
  for(i = 1023; i >  0; i--)
  {
   if(LG_Array[i] > mode_LG)
    {
    mode_LG_index = i;
    mode_LG = LG_Array[i];
    }
    if(HG_Array[i] > mode_HG)
    {
    mode_HG_index = i;
    mode_HG = HG_Array[i];
    }
    /*Create comma seperated strings of spectrum data */
    LG_Buffer_Index += sprintf(&LG_Out_Buffer[LG_Buffer_Index], "%d,", LG_Array[i]); 
    HG_Buffer_Index += sprintf(&HG_Out_Buffer[HG_Buffer_Index], "%d,", HG_Array[i]); 
  }

  
  for(i = 0; i < 255; i++)
  {
  LG_Small_Buffer_Index += sprintf(&LG_Out_Small_Buffer[LG_Small_Buffer_Index], "%d,", LG_Small_Array[i]);
  HG_Small_Buffer_Index += sprintf(&HG_Out_Small_Buffer[HG_Small_Buffer_Index], "%d,", HG_Small_Array[i]);
  }
  
  //LaserI = analogRead(LASER_I_MON)/4095.0 * 3.191*2.0; //Read the laser feedback
  
  Get_Baseline(128);

  LG_Out_Buffer[LG_Buffer_Index + 1] = '\0';  //Add a null to the end of each array so we can treat them as strings
  HG_Out_Buffer[HG_Buffer_Index + 1] = '\0';

  TimeStamp = String(millis()); //Grab a timestamp so we can align with HK data. 

  /*Assemble to string to wrtie to SD Card */
  OutputString += TimeStamp;
  OutputString += ",";
  OutputString += String(LaserI);
  OutputString += ",";
  OutputString += String(Threshold);
  OutputString += ",";
  OutputString += String(LG_Baseline);
  OutputString += ",";
  OutputString += String(HG_Baseline);
  OutputString += ",LG,";
  OutputString += LG_Out_Buffer;
  OutputString += "HG,";
  OutputString += HG_Out_Buffer;
  OutputString.replace(" ","");
  OutputString += "\n";
  
  /*Write data line to SD card */
  PHA.WriteData(file, OutputString);
  OutputString = ""; //Clear output string
  
  /* Write data to USB port */
  // DEBUG_SERIAL.println(TimeStamp + " " + pulse_count + " " + LG_Offset + " " + HG_Offset + " "+ LaserI + " " + Threshold);
  // DEBUG_SERIAL.print("Low Gain Bins (1024): ");
  // DEBUG_SERIAL.write(LG_Out_Buffer, LG_Buffer_Index);
  // DEBUG_SERIAL.flush();
  // DEBUG_SERIAL.println();
  // DEBUG_SERIAL.print("High Gain Bins (1024): ");
  // DEBUG_SERIAL.write(HG_Out_Buffer, HG_Buffer_Index);
  // DEBUG_SERIAL.flush();
  // DEBUG_SERIAL.println(" ");

  
  /*Write reduced data array to TTL serial to Main Board*/

  Serial1.print(TimeStamp);
  Serial1.print(",");
  Serial1.print(LaserI);
  Serial1.print(",");
  Serial1.print(Threshold);
  Serial1.print(",");
  Serial1.print(pulse_count);
  Serial1.print(",");
  Serial1.write((const uint8_t *)HG_Out_Small_Buffer,HG_Small_Buffer_Index);
  Serial1.flush();
  Serial1.write((const uint8_t *)LG_Out_Small_Buffer,LG_Small_Buffer_Index);
  Serial1.flush();
  Serial1.print('E');
  Serial1.print('\n');
  Serial1.flush();
  
/*
  if (pulse_count > SATURATION)
   {
    Threshold -= 10;  //drop the threshold by 10
    analogWrite(HG_THRESH_SET,Threshold); //update the LG threshold
   }
   */
  /* This is debug code */

#ifdef INTERACTIVE
 
  while (Serial.available()) {
   C = Serial.read();
   Serial.print(C);
   if((C == '\n') || (C == '\r'))
   {
    parseCommand(Input_Buff);
    Input_Buff = "";
   } else
   {
    Input_Buff += C;
   }   
  }

  while (OUTPUT_SERIAL.available()) {
   C = OUTPUT_SERIAL.read();
   if((C == '\n') || (C == '\r'))
   {
    parseCommand(Input_Buff);
    Input_Buff = "";
   } else
   {
    Input_Buff += C;
   }   
  }


#endif
  
  DEBUG_SERIAL.print("Pulse Count: ");
  DEBUG_SERIAL.print(pulse_count);
  DEBUG_SERIAL.print(" Low Gain Peak Bin: ");
  DEBUG_SERIAL.print(1023 - mode_LG_index);
  DEBUG_SERIAL.print(" Volts: ");
  //= -6.801E-03x + 7.156E+00 (PHA #2)
  //= 5.526E-03x - 7.138E-02 (PHA revB #4)
  //V Low Gain = 5.343e-3*Bits - 1.146e-1 (PHA Rev C S/N 003)
  DEBUG_SERIAL.print( 5.343E-03*(1023 - mode_LG_index) - 1.146e-1,2);
  DEBUG_SERIAL.print(" High Gain Peak Bin: ");
  DEBUG_SERIAL.print(1023 - mode_HG_index);
   DEBUG_SERIAL.print(" Volts: ");
  //= -1.231E-03x + 1.237E+00 (PHA #2)
  //6.974E-04x - 3.190E-03 (PHA revB #4)
  //V High Gain = 6.814e-4*Bits+ 1.152e-2 (PHA Rev C S/N 003)
  DEBUG_SERIAL.print( 6.814e-4*(1023-mode_HG_index) - 1.152e-2,3);
  DEBUG_SERIAL.print(" HG Threshold: ");
  DEBUG_SERIAL.print(Threshold);
  DEBUG_SERIAL.print(" HG Baseline: ");
  DEBUG_SERIAL.print(HG_Baseline);
  DEBUG_SERIAL.print(" LG Baseline: ");
  DEBUG_SERIAL.println(LG_Baseline);
  int timeEnd = micros(); 
  int delta = timeEnd - timeStart;  // time counts down; subtract 2 for timing instructions
  
  DEBUG_SERIAL.println(" ");
  DEBUG_SERIAL.print("Dead Time (millis): ");
  DEBUG_SERIAL.println(delta/1000.0);
  
 /*Reset counters and arrays */
  pulse_count = 0;
  lg_bad_pulse = 0;
  hg_bad_pulse = 0;
  HG_Offset = 0;
  LG_Offset = 0;
  memset(HG_Array,0,sizeof(HG_Array));
  memset(LG_Array,0,sizeof(LG_Array));
  memset(HG_Small_Array,0,sizeof(HG_Small_Array));
  memset(LG_Small_Array,0,sizeof(LG_Small_Array));
  startTime = millis();
  
}
}



//***********************************************************************************************
//***********************************************************************************************
//
//  -- Run this function only when no pulses are being generarted (or when TP12 = GND)
//  -- takes two ADC clock cycles to start up the data pipe line
//  -- HG and LG ADC clocks are 180 deg out of phase to make sampling as fast as poosible
//  -- ADC samples analog value on negative clock edge, data valid 25ns after negative edge
//  -- Low Gain ADC is on Port D, High Gain ADC is on Port C
//
//***********************************************************************************************
//***********************************************************************************************
FASTRUN void Get_Baseline(int NumSamples)
{
  cli();  // disable interrupts
  HG_Baseline = 0;
  LG_Baseline = 0;

  digitalWriteFast(HG_ADC_CLK, LOW);  digitalWriteFast(LG_ADC_CLK, HIGH);
  digitalWriteFast(HG_ADC_CLK, HIGH); digitalWriteFast(LG_ADC_CLK, LOW);  
  
  for(int i = 0; i < NumSamples; i++)
  {
    digitalWriteFast(HG_ADC_OE, HIGH);
    digitalWriteFast(LG_ADC_OE, LOW);
    delayNanoseconds(1);
    digitalWriteFast(LG_ADC_CLK, LOW);
    digitalWriteFast(HG_ADC_CLK, HIGH);
    LG_Baseline += (GPIO6_PSR & 0x03FF0000) >> 16;
    digitalWriteFast(LG_ADC_OE, HIGH);
    digitalWriteFast(HG_ADC_OE, LOW);
    delayNanoseconds(1);
    digitalWriteFast(HG_ADC_CLK, LOW);
    digitalWriteFast(LG_ADC_CLK, HIGH);
    HG_Baseline += (GPIO6_PSR & 0x03FF0000) >> 16;
  }
  

  digitalWriteFast(LG_ADC_CLK, HIGH);
  sei();  // enable interrupts
  
  HG_Baseline = HG_Baseline/NumSamples;
  LG_Baseline = LG_Baseline/NumSamples;
}



FASTRUN int Get_Pulse_fast(unsigned int trigger_level)
{
  unsigned long TimeOut = millis() + 2000;
  uint16_t dummy, result;
  int i = 0;

  // Run the ADCs a few times to clear the pipeline, leave the OEs high b/c we don't care about the result
  digitalWriteFast(HG_ADC_OE, HIGH); digitalWriteFast(LG_ADC_OE, HIGH); 
  digitalWriteFast(HG_ADC_CLK, LOW);  digitalWriteFast(LG_ADC_CLK, HIGH); dummy = (GPIO6_PSR & 0x03FF0000) >> 16;
  digitalWriteFast(HG_ADC_CLK, HIGH); digitalWriteFast(LG_ADC_CLK, LOW);  dummy = (GPIO6_PSR & 0x03FF0000) >> 16;
  digitalWriteFast(HG_ADC_CLK, LOW);  digitalWriteFast(LG_ADC_CLK, HIGH); dummy = (GPIO6_PSR & 0x03FF0000) >> 16;
  digitalWriteFast(HG_ADC_CLK, HIGH); digitalWriteFast(LG_ADC_CLK, LOW);  dummy = (GPIO6_PSR & 0x03FF0000) >> 16;

  /*Make sure we are not triggered before we look for a pulse */
  result = 0;
  while(result < trigger_level)
  {
    /* Read the HG Channel */
    digitalWriteFast(LG_ADC_OE, HIGH);
    digitalWriteFast(HG_ADC_OE, LOW);
    delayNanoseconds(5);
    digitalWriteFast(HG_ADC_CLK, LOW);
    digitalWriteFast(LG_ADC_CLK, HIGH);
    result = (GPIO6_PSR & 0x03FF0000) >> 16;
    /* Read and ignore low gain channel to keep the clock symmetrical */
    digitalWriteFast(HG_ADC_OE, HIGH);
    digitalWriteFast(LG_ADC_OE, LOW);
    delayNanoseconds(5);
    digitalWriteFast(LG_ADC_CLK, LOW);
    digitalWriteFast(HG_ADC_CLK, HIGH);
    dummy = (GPIO6_PSR & 0x03FF0000) >> 16;
    if(millis() > TimeOut)
      {
        DEBUG_SERIAL.println("Quiescent signal above trigger level!");
        return -1;
      }
  }

/* Wait for a pulse, or a timeout */
  while(1)
  {
   /* Read the HG Channel */
    digitalWriteFast(LG_ADC_OE, HIGH);
    digitalWriteFast(HG_ADC_OE, LOW);
    delayNanoseconds(1);
    digitalWriteFast(HG_ADC_CLK, LOW);
    digitalWriteFast(LG_ADC_CLK, HIGH);
    result = (GPIO6_PSR & 0x03FF0000) >> 16;
    if( result < trigger_level) //if we are above the trigger level stop
           break;

    /* Read and ignore low gain channel to keep the clock symmetrical */
    digitalWriteFast(HG_ADC_OE, HIGH);
    digitalWriteFast(LG_ADC_OE, LOW);
    delayNanoseconds(1);
    digitalWriteFast(LG_ADC_CLK, LOW);
    digitalWriteFast(HG_ADC_CLK, HIGH);
    dummy = (GPIO6_PSR & 0x03FF0000) >> 16; 
    if(millis() > TimeOut)
      return -1;
  }

  /* Digitize the pulse on both channels */
  cli();
  digitalWriteFast(TRIGGERED, HIGH);  
  for(i = PULSE_WINDOW; i > 0; i--)
  {
    
    digitalWriteFast(HG_ADC_OE, HIGH);
    digitalWriteFast(LG_ADC_OE, LOW);
    delayNanoseconds(1);
    digitalWriteFast(LG_ADC_CLK, LOW);
    digitalWriteFast(HG_ADC_CLK, HIGH);
    LG_buff[i] = (GPIO6_PSR & 0x03FF0000) >> 16;
    digitalWriteFast(LG_ADC_OE, HIGH);
    digitalWriteFast(HG_ADC_OE, LOW);
    delayNanoseconds(1);
    digitalWriteFast(HG_ADC_CLK, LOW);
    digitalWriteFast(LG_ADC_CLK, HIGH);
    HG_buff[i] = (GPIO6_PSR & 0x03FF0000) >> 16;
  }
  digitalWriteFast(TRIGGERED, LOW); 
  sei();
  return 1;
}



void parseCommand(String commandToParse)
{
  /* This is where all the commands are interpreted and is the meat of the controll system
   * so far
   * 
   * #save - write the settings to EEPROM
   * #clear - clear the settings from EEPROM (this resets to hard coded default)
   */
  
  char * strtokIndx; // this is used by strtok() as an index
  char CommandArray[64];
  int int1 = 0;
  
  
  if(commandToParse.startsWith("#hgoff"))
  {
     commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
     strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
  
     strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
     int1 = atoi(strtokIndx);     // convert this part to a float for the temperature set point

     DEBUG_SERIAL.print("Setting High Gain Baseline Offset to: ");
     DEBUG_SERIAL.print(int1); DEBUG_SERIAL.println(" [Bits]");

     HG_Offset_val = int1; //Set point for pump 1

     setOutputB(HG_Offset_val);
     
     commandToParse = "";
  }
  else if(commandToParse.startsWith("#lgoff"))
  {
     commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
     strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
  
     strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
     int1 = atoi(strtokIndx);     // convert this part to a float for the temperature set point

     DEBUG_SERIAL.print("Setting Low Gain Baseline Offset to: ");
     DEBUG_SERIAL.print(int1); DEBUG_SERIAL.println(" [Bits]");

     LG_Offset_val = int1; 

     setOutputA(LG_Offset_val);      // 12 bit DAC output 0V to 1.2V, (3600/4096)*1.2V = 1.055V     
     commandToParse = "";
  }
  else if(commandToParse.startsWith("#thre"))
  {
     commandToParse.toCharArray(CommandArray, 64); //copy the String() to a string
     strtokIndx = strtok(CommandArray,",");      // get the first part - the string we don't care about this
  
     strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
     int1 = atoi(strtokIndx);     // convert this part to a float for the temperature set point

     DEBUG_SERIAL.print("Setting Pulse Detection Threshold to: ");
     DEBUG_SERIAL.print(int1); DEBUG_SERIAL.println(" [Bits]");

     Threshold = int1; //Pulse threshold
     
     commandToParse = "";
  }
  else if(commandToParse.startsWith("#print"))
  {
     DEBUG_SERIAL.println();
     DEBUG_SERIAL.println("Current Parameters, note these may not be saved");
     DEBUG_SERIAL.print("High Gain Offset: "); DEBUG_SERIAL.println(HG_Offset_val);
     DEBUG_SERIAL.print("Low Gain Offset: "); DEBUG_SERIAL.println(LG_Offset_val);
     DEBUG_SERIAL.print("Pulse Detection Threshold: "); DEBUG_SERIAL.println(Threshold);
     DEBUG_SERIAL.println();
  }
 
  else if(commandToParse.startsWith("#save"))
  {
     DEBUG_SERIAL.println("Writing Settings to EEPROM");
     EEPROM.put(EEPROM_HG_OFFSET, HG_Offset_val); //save HG Offset as int
     EEPROM.put(EEPROM_LG_OFFSET, LG_Offset_val); //save LG Offset as int
     EEPROM.put(EEPROM_THRESHOLD, Threshold); //save Threshold as int
     EEPROM.write(EEPROM_FLAG, 0x01);    //update flag to show valid data in eeprom
  }
  else if(commandToParse.startsWith("#clear"))
  {
     DEBUG_SERIAL.println("Clearing settings from EEPROM");
     EEPROM.put(EEPROM_HG_OFFSET, 0); //clear HG Offset as int
     EEPROM.put(EEPROM_LG_OFFSET, 0); //clear LG Offset as int
     EEPROM.put(EEPROM_THRESHOLD, 0); //clear Threshold as int 
     EEPROM.write(EEPROM_FLAG, 0x00);    //update flag to show no valid data in eeprom
  }
  else
  {
     DEBUG_SERIAL.println();
     DEBUG_SERIAL.println();
     DEBUG_SERIAL.println("Command not recognized, valid commands: ");
     DEBUG_SERIAL.println("#lgoff, int - low gain offset in bits (0 - 4095)");
     DEBUG_SERIAL.println("#hgoff, int - high gain offset in bits (0 - 4095)");
     DEBUG_SERIAL.println("#thresh, int - pulse detection threshold in bits (0 - 1024)");
     DEBUG_SERIAL.println("#print - display the current settings");
     DEBUG_SERIAL.println("#save, save the settings to EEPROM");
     DEBUG_SERIAL.println("#clear, clear the settings from EEPROM");
     DEBUG_SERIAL.println();
     commandToParse = "";
  }
}

void setDACReference(int ref)
{
  byte refbyte;
  if(ref == 0) //Vdd
    refbyte = 0x0;
  if(ref == 1) //1.2v bandgap
    refbyte = 0x05;
  if(ref == 2) //Ext unbuffered
    refbyte = 0x0A;
  if(ref == 3) //Ext buffered
    refbyte = 0x0F;
  
  digitalWrite(DAC_CS, LOW);
  SPI.transfer(0x40);
  SPI.transfer(0x00);
  SPI.transfer(refbyte);
  digitalWrite(DAC_CS,HIGH);
}

void setDACGain(byte Gain)
{
  digitalWrite(DAC_CS, LOW);
  SPI.transfer(0x0A<<3);
  SPI.transfer(Gain);
  SPI.transfer(0x00);
  digitalWrite(DAC_CS,HIGH);
}

void setOutputA(unsigned int val)
{
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff);
  
  digitalWrite(DAC_CS, LOW);
  SPI.transfer(0x00);
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  digitalWrite(DAC_CS,HIGH);
}

void setOutputB(unsigned int val)
{
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff);
  
  digitalWrite(DAC_CS, LOW);
  SPI.transfer(0x08);
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  digitalWrite(DAC_CS,HIGH);
}


