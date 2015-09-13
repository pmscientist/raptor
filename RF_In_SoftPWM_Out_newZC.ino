/*

 SoftPWM SCR driver for the Raptor12
 
 SoftPWM is an all in one PWM driver for 12 channels based on a Atmega328 chipset.  It supports
 zerocross detection as well as simulated ZC for use as a DC based controller as well as an NON-Isolated
 AC based contoroller.
 
 Created by Travis Kneale   2014
 Adapted from sketch by by Robert Twomey <rtwomey@u.washington.edu>
 Adapted from sketch by Ryan McLaughlin <ryanjmclaughlin@gmail.com> 
 
 Modified 7/9/2014 Greg Scull - komby@komby.com to work with Clarity release 
 
 
 */

#include <Arduino.h>
#include <RFShowControl.h>
#include <IRFShowControl.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <TimerOne.h>


/*************************** CONFIGURATION SECTION *************************************************/
// RECEIVER_UNIQUE_ID Description: http://learn.komby.com/wiki/58/configuration-settings#RECEIVER_UNIQUE_ID
// Valid Values: 1-255
#define RECEIVER_UNIQUE_ID 33

// OVER_THE_AIR_CONFIG_ENABLE Description: http://learn.komby.com/wiki/58/configuration-settings#OVER_THE_AIR_CONFIG_ENABLE
// Valid Values: OTA_ENABLED, OTA_DISABLED
#define OVER_THE_AIR_CONFIG_ENABLE      0

// NRF_TYPE Description: http://learn.komby.com/wiki/58/configuration-settings#NRF_TYPE
// Valid Values: RAPTOR12
#define NRF_TYPE  RAPTOR12

// NRF_TYPE Description: http://learn.komby.com/wiki/58/configuration-settings#AC_DC
// Valid Values: 1, 0
#define AC_DC  1


/********************** END OF REQUIRED CONFIGURATION ************************/

/****************** START OF NON-OTA CONFIGURATION SECTION *******************/
// LISTEN_CHANNEL Description: http://learn.komby.com/wiki/58/configuration-settings#LISTEN_CHANNEL
// Valid Values: 0-83, 101-127  (Note: use of channels 84-100 is not allowed in the US)
#define LISTEN_CHANNEL                  0

// DATA_RATE Description: http://learn.komby.com/wiki/58/configuration-settings#DATA_RATE
// Valid Values: RF24_250KBPS, RF24_1MBPS
#define DATA_RATE                       RF24_250KBPS

// HARDCODED_START_CHANNEL Description: http://learn.komby.com/wiki/58/configuration-settings#HARDCODED_START_CHANNEL
// Valid Values: 1-512
#define HARDCODED_START_CHANNEL         1

// HARDCODED_NUM_CHANNELS Description: http://learn.komby.com/wiki/58/configuration-settings#HARDCODED_NUM_CHANNELS
// Valid Values: 1
// strobe control only uses one channel
#define HARDCODED_NUM_CHANNELS          12

/******************* END OF NON-OTA CONFIGURATION SECTION ********************/


/************** START OF ADVANCED SETTINGS SECTION (OPTIONAL) ****************/
#define PIXEL_TYPE NONE


#define FINAL_CHANNEL 12 //DO Refactor out, addition would be easy.... 
 
// Just guess and check at the moment  
// 1000000 uS / 120 Hz ) / (256 + 60) = 26(32old)brightness steps = 16.25
// 1000000 uS / 120 Hz ) / (255) = 32.6
#define freqStep  31

//FCC_RESTRICT Description: http://learn.komby.com/wiki/58/configuration-settings#FCC_RESTRICT
//Valid Values: 1, 0  (1 will prevent the use of channels that are not allowed in North America)
#define FCC_RESTRICT 1

 /**************END CONFIGURATION SECTION ***************************/
//Include this after all configuration variables are set

#include "RFShowControlConfig.h"

bool readytoupdate=false;

byte * buffer;
byte * bufferOutput;

//Uncomment for serial
//#define DEBUG 0

 
// int totalChannel = 12;
 
  volatile uint8_t dimLevel=255;               // Variable to use as a counter
  volatile boolean zeroCross = false;  // Boolean to store a "switch" to tell us if we have crossed zero
//  int lastLevel;                         //This is for the Triac firing function to do a bit of math
  volatile uint8_t dataLedCycle = 0;  //set a 1 sec blink cycle
  volatile uint8_t dataAvaliable = 0;

void setup() {  // Begin setup  
  //prevent interrupts while setting up
  cli();
  
  pinMode(2, INPUT);
  attachInterrupt(0, zeroCrossDetect, FALLING);      // Attach an Interrupt to Pin 2 (interrupt 0) for Zero Cross Detection
  
  Timer1.initialize(freqStep);                           // Initialize TimerOne library for the freq we need
  Timer1.attachInterrupt(triggerOutputPin, freqStep);      // See if we can fire the SCR.
  Timer1.stop();                                         // Stop timer for now to sync w/ZC
   
  //Setup output port pins
  // B0    C0 C1 C2 C3 C4 C5    D3 D4 D5 D6 D7
  //using OR to make only the correct pins are modified.
  DDRB = DDRB | B00000001; 
  DDRC = DDRC | B00111111;
  DDRD = DDRD | B11111000; 
  
  #ifdef DEBUG
     Serial.begin(115200);
  #else 
     //Staus leds are disabled if debug is on
     DDRD = DDRD | B00000011;
  #endif 
  

  buffer[0]=255;
 
  radio.EnableOverTheAirConfiguration(OVER_THE_AIR_CONFIG_ENABLE);
  if(!OVER_THE_AIR_CONFIG_ENABLE)
     {
        int logicalControllerSequenceNum = 0;
        radio.AddLogicalController(logicalControllerSequenceNum, HARDCODED_START_CHANNEL, HARDCODED_NUM_CHANNELS,0);
     }
	
  delay(2);
  
  //this detects if radio is accely functioning if not the led will stay off.
  if(radio.Initialize( radio.RECEIVER, pipes, LISTEN_CHANNEL,DATA_RATE ,RECEIVER_UNIQUE_ID)){
    PORTD =  PORTD | B00000010;
  }
  
  #ifdef DEBUG
    radio.printDetails();
  #endif 
  //initialize data buffer
  buffer= radio.GetControllerDataBase(0);	
  delay (200); //needed or leftover code?
  
  //re-enable interrupts
  sei();
}


void triggerOutputPin(){
  if (zeroCross) {
    zeroCross = false;

    //lazy people cut and paste and so do I 
    //I apologise to anyone the wants to remap pins:P (heres a hint dont do it)
    //This will flip on any channels that may be set to full brightness.
    //less noise not that it matters mutch at all

    //Channel 1
    if (buffer[0] == 255 ){
      PORTB = PORTB | B00000001; 
    } else {
      PORTB = PORTB & B11111110;
    }
    //Channel 2
    if (buffer[1] == 255){
      PORTC = PORTC | B00000001;
    } else {
      PORTC = PORTC & B11111110;
    }
    //Channel 3
    if (buffer[2] == 255){
      PORTC = PORTC | B00000010;
    } else {
      PORTC = PORTC & B11111101;
    }
    //Channel 4
    if (buffer[3] == 255){
      PORTC = PORTC | B00000100;
    } else {
      PORTC = PORTC & B11111011;
    }
    //Channel 5
    if (buffer[4] == 255){      
      PORTC = PORTC | B00001000;
    } else {
      PORTC = PORTC & B11110111;
    }
    //Channel 6
    if (buffer[5] == 255){        
      PORTC = PORTC | B00010000;
    } else {
      PORTC = PORTC & B11101111;
    }
    //Channel 7
    if (buffer[6] == 255){        
      PORTC = PORTC | B00100000;        
    } else {
      PORTC = PORTC & B11011111;
    }
    //Channel 8
    if (buffer[7] == 255){
      PORTD = PORTD | B00001000; 
    } else {
      PORTD = PORTD & B11110111;
    }
    //Channel 9
    if (buffer[8] == 255){
      PORTD = PORTD | B00010000;
    } else {
      PORTD = PORTD & B11101111;
    }
    //Channel 10
    if (buffer[9] == 255){
      PORTD = PORTD | B00100000; 
    } else {
      PORTD = PORTD & B11011111;
    }
    //Channel 11
    if (buffer[10] == 255){
      PORTD = PORTD | B01000000; 
    } else {
      PORTD = PORTD & B10111111;
    }
    //Channel 12
    if (buffer[11] == 255){
      PORTD = PORTD | B10000000;
    } else {
      PORTD = PORTD & B01111111;
    }
   
    dimLevel = 254;

    //start another software pwm cycle
    Timer1.start();
  } else if (dimLevel == 0) {
    //stop software pwm until next ZC
    Timer1.stop();

    //flips leds on and off as needs based on interupt
    ledDataToggle();
  } else {
    //turn on channels if pwm value met
    
    ///Channel 1
    if (buffer[0] >= dimLevel){
      PORTB = PORTB | B00000001; 
    }//Channel 2
    if (buffer[1] >= dimLevel){
      PORTC = PORTC | B00000001;
    }//Channel 3
    if (buffer[2] >= dimLevel){
      PORTC = PORTC | B00000010;
    }//Channel 4
    if (buffer[3] >= dimLevel){
      PORTC = PORTC | B00000100;
    }//Channel 5
    if (buffer[4] >= dimLevel){      
      PORTC = PORTC | B00001000;
    }//Channel 6
    if (buffer[5] >= dimLevel){        
      PORTC = PORTC | B00010000;
    }//Channel 7
    if (buffer[6] >= dimLevel){        
      PORTC = PORTC | B00100000;        
    }//Channel 8
    if (buffer[7] >= dimLevel){
      PORTD = PORTD | B00001000; 
    }//Channel 9
    if (buffer[8] >= dimLevel){
      PORTD = PORTD | B00010000;
    }//Channel 10
    if (buffer[9] >= dimLevel){
      PORTD = PORTD | B00100000; 
    }//Channel 11
    if (buffer[10] >= dimLevel){
      PORTD = PORTD | B01000000; 
    }//Channel 12
    if (buffer[11] >= dimLevel){
      PORTD = PORTD | B10000000;
    }
    
    --dimLevel;
  }
}

void zeroCrossDetect() {
    zeroCross = true;
}


//When ever one of the dimming points are reached run this to enable pins
/* void zeroCrossEvent() {  
  if (dimLevel > 0){ //this will prevent this var going negative
    dimLevel = dimLevel - 1; // decrease time step counter 

  }else{
    //if we missed the zero cross fake where it should be
    if (lastLevel >= 30){ 
     zeroCrossDetect();
      dimLevel = dimLevel - lastLevel;
      lastLevel = 0;
    }
    lastLevel = lastLevel + 1;
  }
  //accely do the dimming after we check the importain stuff
 triggerOutputPin(); 
} */

void ledDataToggle(){
  if  (dataAvaliable != 0){
    if (dataLedCycle >= 40){
      PORTD =  PORTD | B00000001;
    }else{
      PORTD =  PORTD & B11111110;
    }
    --dataLedCycle;
    --dataAvaliable;
    if (dataLedCycle == 0){
      dataLedCycle = 80;
    }
  }
}

//Main loop that runs when nothing is going on with interupts
void loop() {
  if(radio.Listen()){
    bufferOutput = buffer;
    dataAvaliable = 120; //reset couter when we get data.
  }
  
  if (zeroCross)
    triggerOutputPin();
}
