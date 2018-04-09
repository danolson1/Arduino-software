#include <MemoryUsage.h> // for troubleshooting freezing

/*
Internet OD reader (IODR) program
Detect growth of bacterial cultures by measuring the absorbtion of a light
Version 22
Dan Olson
1-18-2018

New in v22
- changing over from xively to thingspeak for storing cloud data, since xively just shut down
- I'm using the ThingSpeak account associated with my daniel.g.olson@dartmouth.edu account

New in v21
-still having intermittent freezing problems, trying to troubleshoot.

For use with v1 of the IODR, 
 -using an Arduino Mega 2560 
 -Arduino ethernet shield (replaced Adafruit CC3000 WiFi shield, which had stability problems)
 -IODR v2 shield circuit board (designed by me)
 -note:  the Spol jumper on the OLED display board needs to be cut in order for the display to function properly with the Arduino

Pin assignments:
0: used by ethernet shield
1: used by ethernet shield
2: yellow LED
4: ethernet CS for SD card (not used in this program)
5: previously used for entering calibration mode (note that the calibration mode hasn't been implemented in this version of the software)
7: unused (previous version used this pin for wifi shiled)
8: temperature sensor
9: transistor that controls all 8 LEDs (note, this was previously pin 7, but moved to pin 9 since the wifi shield needs to use pin 7)
10: I think this is used for the ethernet shield, output is always high
18: TX for lcd display

Pins for LED shield:
12
11
3
6
7


Ethernet shield connected to SPI bus (using SPI header instead of dedicated pins)
53: SS for ethernet controller
51: ethernet MOSI
50: ethernet MISO
52: ethernet SCK

53: necessary to set this pin as output to allow ethernet shield to work
20-34 (even numbered pins): blank reset buttons
A8: pin 89, light sensor for tube 8
A9: pin 88, light sensor for tube 7
A10: pin 87, light sensor for tube 6
A11: pin 86, light sensor for tube 5
A12: pin 85, light sensor for tube 4
A13: pin 84, light sensor for tube 3
A14: pin 83, light sensor for tube 2
A15: pin 82, light sensor for tube 1
*/

//software version
#define VERSION 22

//used for temperature sensor 
#define DS18S20_ID 0x10
#define DS18B20_ID 0x28

// Libraries
#include <Ethernet2.h> //ethernet shield
#include <SPI.h> //communication with wifi or ethernet shield
#include <avr/wdt.h> //for watchdog timer
#include <OneWire.h> //for temperature sensor
#include <HttpClient.h>
#include <Math.h>
#include <SoftwareSerial.h> //for serial LCD communication
#include "ThingSpeak.h"
#include <EEPROM.h> //used to store blank values so they persist between resets
#include <gloSerialOLED.h> //for OLED character display
gloStreamingTemplate //allows carat notation for OLED display

// ThingSpeak parameters for IODR #1
#define OD_CHANNEL_ID 405675
#define OD_API_KEY "J1AW4IF7557WHYAX" //write API key
#define TEMP_CHANNEL_ID 405681
#define TEMP_API_KEY "UTQMZ1FXORYGECIY" //write API key

// ThingSpeak parameters for IODR #2
//#define OD_CHANNEL_ID 441742
//#define OD_API_KEY "P212KDUXZPL5BAFA" //write API key
//#define TEMP_CHANNEL_ID 441744
//#define TEMP_API_KEY "5C8JTN6LJ1CKVGT6" //write API key


unsigned long lastConnectionTime = 0;                // last time we connected to Thingspeak
const unsigned long connectionInterval = 60000;      // delay between connecting to Thingspeak in milliseconds
unsigned long lastLoopTime = 0;                      // to keep track of the loop speed
unsigned long lastODReadTime = 0;                    // last time the light sensor was read
unsigned long connectionIntervalODRead = 800;        // delay between OD reads, also delay between temperature reads
const int MAX_FAILED_UPLOADS = 8;             // reset the arduino if there are too many failed upload attempts
int numFailedUploads = 0;                      // keep track of how many times the data upload has failed (i.e. server did not return 200)

// digital temperature sensor
int DS18S20_Pin = 8; //DS18S20 temp sensor signal pin on digital 8
OneWire ds(DS18S20_Pin); // on digital pin 8
float temperature;
byte tempSensAddr[8];
boolean temperatureDeviceFound = false;


// LEDs and light sensors
int ledPin = 9;
int yellowLED = 2;
int numTubes = 8;

int lightInPin[] = {A15, A14, A13, A12, A11, A10, A9, A8}; //pins for analog inputs for light sensor
float lightIn[] = {0,0,0,0,0,0,0,0}; //value of light sensor
float ODvalue[] = {0,0,0,0,0,0,0,0};
int LEDoffReading[] = {0,0,0,0,0,0,0,0};
int LEDonReading[] = {0,0,0,0,0,0,0,0};
int pointsToAverage = 10; //number of light readings to average in each data point sent to Cosm
int blankValue[] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; //default blank value of 1000 for each light sensor

int blankButtonState[] = {0,0,0,0,0,0,0,0}; // status of buttons for blanking individual tubes
int blankButtonPin[] = {20, 22, 24, 26, 28, 30, 32, 34}; 
int lastButtonPressed = 0; // keep track of which reset button was pressed most recently
const int NUM_CYCLES_TO_RESET = 12; //number of cycles the blank button has to be pressed down to before the blank value is reset

int calibrationPin = 5;
int calibrationMode = 1; // 1 is for no calibration, 0 is for calibration mode.  in calibration mode, one reading is taken every time the button is pressed
int calibrationCount = 0; // keeps track of how many times the calibration button was pressed

// for lcd serial display
//SoftwareSerial mySerial (3,18);
gloSoftSerialSetup( myOLED, 18, noninverted );
boolean displayTubeSummary = true;


// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 80 is default for HTTP):
EthernetClient client;

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { 0x2C, 0xF7, 0xF1, 0x08, 0x10, 0x4A }; //mac address for IODR #1 new ethernet shield W5500
//byte mac[] = { 0x90, 0xA2, 0xDA, 0x0D, 0x45, 0x10 }; //mac address for IODR #1 old ethernet shield
//byte mac[] = { 0x90, 0xA2, 0xDA, 0x0D, 0xA7, 0x42 }; //mac address for IODR #2

// for LED shield
int setupLED = 7;
int startMainLoopLED = 6;
int readLightSensorLED = 3;
int getTempLED = 11;
int startTempLED = 12;




//------------------------------------------------------------------------
//           SETUP
//------------------------------------------------------------------------
void setup(void)
{
  //enable the watchdog timer at the beginning of the setup loop
  //if the program hangs for more than 8 seconds, the arduino will be reset
  //when the program is functioning correctly, the watchdog timer needs to be reset more frequently than every 8 seconds
  digitalWrite(setupLED, HIGH);
  wdt_enable(WDTO_8S);
  
  // set up DS18B20 digital thermometer
  temperatureDeviceFound = initTemp();
  startTempConversion(); //send the first request to measure temperature
  
  //calibration mode (might add this in later if I need it)
  
  //read blankValues from EEPROM and store in blankValue integer array
  //these values are saved between resets
   for (int i=0; i < numTubes; i++){
     readBlankFromEEPROM(i);
   }
  
  //set up serial LCD display
  // Begin using the serial object created above.
  gloSoftSerialBegin( myOLED ) ;
  //mySerial.begin(9600); //start lcd display (old LCD display)
  myOLED << gloClear << gloFont_1w1h ;
  delay(500); // wait for lcd display to boot up
  
  //set up I/O pins
  pinMode(ledPin, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  analogReference(EXTERNAL);
  pinMode(calibrationPin, INPUT);
  pinMode(53, OUTPUT); //necessary for WiFi shield function
  
  //set up pins for LED shield
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  
  //announce version information
  Serial.begin(115200); //serial port for debugging set to 115200 baud
  Serial.print("Internet OD reader v");
  Serial.println(VERSION);
  Serial.println("==========================");
  myOLED <<gloClear<<"Internet OD reader v" << VERSION;
  wdt_reset();
  
  // Initialize ethernet
  Serial.println(F("Initializing ethernet chip..."));
  // send message to OLED display
  myOLED << gloReturn << "connecting to internet..."; 
  
  if (!Ethernet.begin(mac))
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    myOLED << gloClear << "error, no internet connection, please reset";
    while(1);
  }
  
  //display IP address
  Serial.print("IP = ");
  Serial.println(Ethernet.localIP());
  
  myOLED << gloClear << "IP=" << Ethernet.localIP();
  myOLED << gloReturn << "mac=" << mac[0] << ":" << mac[1] << ":" << mac[2] << ":" << mac[3] << ":" << mac[4] << ":" << mac[5];
  Serial.println("     ***** Ethernet.begin() *****");
  checkTiming();
  wdt_reset();
  delay (3000);
  
  Serial.println("Setup complete");
  myOLED << gloClear;
  checkTiming();
  wdt_reset();
  digitalWrite(setupLED, LOW);

  //set up thingspeak
  ThingSpeak.begin(client);
}
//------------------------------------------------------------------------
//           end SETUP
//------------------------------------------------------------------------



//------------------------------------------------------------------------
//           MAIN PROGRAM LOOP
//------------------------------------------------------------------------
void loop(){
  // Reset watchdog
  wdt_reset();
  
  digitalWrite(startMainLoopLED, HIGH);
  
  //check blank buttons.  These are the momentary switches along the top edge of the IODR circuit board
  checkBlankButtons();
  Serial.println("     ***** checkBlankButtons() *****");
  checkTiming();
  
  //display light sensor status on LCD
  if (displayTubeSummary){
     displayTubeStatusSummary();
     Serial.println("     ***** displayTubeStatusSummary() *****");
     checkTiming();  
  } 
  else {  
     displayTubeStatus(lastButtonPressed);
     Serial.println("     ***** displayTubeStatus() *****");
     checkTiming();
  } 

  digitalWrite(startMainLoopLED, LOW);


  //read light sensors and check temperature
  if (millis() - lastODReadTime > connectionIntervalODRead){
    
    digitalWrite(readLightSensorLED, HIGH);
    readLightSensors();
    Serial.println("     ***** readLightSensors() *****");
    checkTiming();
    digitalWrite(readLightSensorLED, LOW);
    
    digitalWrite(getTempLED, HIGH);
    temperature = getTemp(); //read the temperature value from the DS18B20 chip
    Serial.println("     ***** getTemp() *****");
    checkTiming();
    digitalWrite(getTempLED, LOW);
    
    digitalWrite(startTempLED, HIGH);
    startTempConversion(); //start the next temperature conversion operation, need to wait >750ms before reading this data
    Serial.println("     ***** startTempConversion() *****");
    checkTiming(); 
    // update time so we wait before reading again
    lastODReadTime = millis();
    digitalWrite(startTempLED, LOW);
    
    //send light sensor, temp and blank button information to serial terminal for debugging purposes
    serialPrintState();
    //blankButtonStatusDisplay();
  }
  
  // send data to Thingspeak
  if (millis() - lastConnectionTime > connectionInterval) { 
    Serial.println("@@@@@ yellow LED on @@@@@");
    digitalWrite(yellowLED, HIGH); // turn on yellow LED when sending data to Thingspeak  
    
    //connect to thingspeak
    Serial.println("@@@@@ uploadDataToThingspeak() @@@@@");
    uploadDataToThingspeak();
    Serial.println("@@@@@ checkTiming() @@@@@");
    checkTiming();  
    // update connection time so we wait before connecting again
    lastConnectionTime = millis();

    Serial.println("***** yellow LED off *****");
    digitalWrite(yellowLED, LOW); // turn off yellow LED
    
    Serial.println("***** display tube summary *****");
    displayTubeSummary = true; // switch display back to summary mode
    Serial.println("***** initTemp *****");
    temperatureDeviceFound = initTemp(); // try to re-initialize the temperature sensor every time data is uploaded, in case there was a problem with the initialization during setup() 
  } 

  FREERAM_PRINT;
  Serial.println("################################# END OF MAIN LOOP #####################################");
  checkTiming();
}
//------------------------------------------------------------------------
//           end MAIN PROGRAM LOOP
//------------------------------------------------------------------------



//write blank value to EEPROM
//8 2-byte values get stored
//address 0 and 1 store the first blank value
//address 2 and 3 store the second blank value
void writeBlankToEEPROM (int tubeIndex){
  Serial << "******************* writeBlankToEEPROM() ******************";
  Serial.print("\nWriting blank value for tube ");
  Serial.print(tubeIndex);
  Serial.println(" from EEPROM");
  int value = blankValue[tubeIndex];
  byte h = highByte(value);
  byte l = lowByte(value);
  EEPROM.write(tubeIndex*2, h);  //store the high byte in the first address
  EEPROM.write(tubeIndex*2+1, l); //store the low byte in the second address
}



//read blank value from EEPROM
void readBlankFromEEPROM (int tubeIndex){
  Serial << "************************** readBlankFromEEPROM() **************************************";
  Serial.print("\nReading blank value for tube ");
  Serial.print(tubeIndex);
  Serial.println(" from EEPROM");
  word value;
  byte h = EEPROM.read(tubeIndex*2); //read low byte from EEPROM
  byte l = EEPROM.read(tubeIndex*2+1); //read high byte from EEPROM
  value = word(h,l);
  blankValue[tubeIndex] = (int)value; //cast value to an integer and store the result in the blankValue array
}



//subroutine for checking the timing of a portion of a loop
void checkTiming(){
  Serial.print("***checkTiming()*** loop time=");
  Serial.print(millis()-lastLoopTime);
  lastLoopTime = millis();
  Serial.println();
}



// check the state of the reset buttons. keep track of which button was most recently pressed.  
// if a button is pressed and held for a certain amount of time (>2 seconds), then reset the blank value for that well
void checkBlankButtons(){
     //Serial << "\n$$$$$$$$$$$ checkBlankButtons() $$$$$$$$$$$$$$$$$";
     for (int i=0; i<numTubes; i++){
      //in v2 of the IODR shield, I accidentally wired the reset buttons to give digital 0 when pressed instead of 
      //digital 1 (as in previous version).  So I added a "!" before the all 3 digitalRead commands to fix this
      blankButtonState[i] += !digitalRead(blankButtonPin[i]); 
      
      // if a button was pressed, keep track of which one
      // if multiple buttons were pressed, remember the highest numbered one
      if (!digitalRead(blankButtonPin[i]) == 1){
        lastButtonPressed = i; 
        Serial << "\nlastButtonPressed=" <<lastButtonPressed;
        displayTubeSummary = false; // switch the OLED display to show data for an individual tube instead of the summary
                                    // note, this will reset back to summary view after the next time data is uploaded to the cloud
      }
      
      // if a button was not held down during this cycle, reset it's counter
      // i.e. reset button 2 was held down for 2 cycles, then it was released
      // now it's held down for 2 cycles again.  it shouldn't reset the blank value
      if (!digitalRead(blankButtonPin[i]) == 0){
        blankButtonState[i] = 0;
      }
      
      // if a button has been held down for more than NUM_CYCLES_TO_RESET cycles, reset that tube
      if (blankButtonState[i] > NUM_CYCLES_TO_RESET) {
        blankValue[i] = (int)lightIn[i]; // reset blankValue for tube i
        Serial << "blankButtonState[" << i << "]=" << blankValue[i];
        writeBlankToEEPROM(i);// write value to EEPROM, this saves the value even if the arduino is reset
        displayTubeReset(i); // write a message to the serial LCD saying the tube was reset
        blankButtonState[i] = 0; // reset the blank button state
      }
    }
    //blankButtonStatusDisplay();
    
    //if the calibration button was pressed, switch the display back to summary mode
    if (!digitalRead(calibrationPin)){
     displayTubeSummary = true;
     Serial << "\n\t\t calibrationPin=" << digitalRead(calibrationPin);
    }
}



// display the state of each pushbutton used to blank the tube reader
void blankButtonStatusDisplay(){
  //Serial << "*************** blankButtonStatusDisplay() ***********************";
  Serial.print("Blank Button State: ");
  for (int i=0; i<numTubes; i++){
     Serial.print(blankButtonState[i]);
     Serial.print(" ");
  }
  Serial.print(" lastButtonPressed=");
  Serial.print(lastButtonPressed);
  Serial.println();
}


//print the state of the light and temperature sensors to the serial monitor for debugging purposes
void serialPrintState(){  
  Serial << "\n**************** serialPrintState() ******************";
  Serial <<"\nLEDoffReading[]\t LEDonReading[]\t lightIn[]\t ODvalue[]\t blankValue";
  for (int i=0; i<numTubes; i++){
     Serial <<"\nTube[" << i << "]:" ;
     Serial.print(LEDoffReading[i]);
     Serial.print("\t");
     Serial.print(LEDonReading[i]);
     Serial.print("\t");
     Serial.print(lightIn[i]); //raw value of light sensor
     Serial.print("\t");
     Serial.print(ODvalue[i]); //calculated OD value
     Serial.print("\t");
     Serial.print(blankValue[i]);
     Serial.print("\t");
  }
  // display temperature   
  Serial.print("\nTemp: ");
  Serial.print(temperature);
  Serial.println();
}



//display a message on the LCD saying that one tube blank value was reset
void displayTubeReset(int tubeNum){
  myOLED << gloClear << "Tube: " << tubeNum + 1; //tubNum index starts at 0, so increment by one for human-readable display
  myOLED << gloReturn << "Was reset" ;
  
  Serial.print("displayTubeReset:");
  //Serial.println(resetTopLine);
  delay(1500); //show the message for 1.5 seconds
}




// Summary Display
// display current OD for all tubes on OLED display
void displayTubeStatusSummary(){
  //Serial.println("*************************************");
  //Serial.println("displayTubeStatusSummary()");
  char displayLine[4][17] = {
       "1:     5:     T ",
       "2:     6:     e ",
       "3:     7:     m ",
       "4:     8:     p "};  
  char odval[5] = "    ";
                           
  //add in OD values to first column
  for (int i=0; i<4; i++){  
    //loop through each character in odval and add it to the displayLine char array
    //add odval in first position (i.e. tubes 1, 2, 3 and 4)
    dtostrf(ODvalue[i], 4, 2, odval);
    //Serial << "   odval[" << i << "]=" << odval <<" length=" << sizeof(odval);
    
    for (int j = 0; j<sizeof(odval)-1; j++){ //sizeof(odval)-1 is to avoid including the "end of string" character
      displayLine[i][j+2] = odval[j];
      //Serial << "        displayLine["<<i<<"]["<<j+2<<"]="<<odval[j];
    }
    //add odval in second position (i.e. tubes 5, 6, 7 and 8)
    dtostrf(ODvalue[i+4], 4, 2, odval);  
    //Serial << "   odval_2_[" << i << "]=" << odval <<" length=" << sizeof(odval);
    //Serial.println();
    for (int j = 0; j<sizeof(odval)-1; j++){
      displayLine[i][j+9] = odval[j];
    }
  }

  //add temperature value
  char tempStr[4];
  dtostrf(temperature, 4,2, tempStr);
  displayLine[0][15] = tempStr[0];
  displayLine[1][15] = tempStr[1];
  displayLine[2][15] = tempStr[2];
  displayLine[3][15] = tempStr[3];

/*  
  Serial.println("\n*************** displayTubeStatusSummary() *****************");
  for (int i=0; i<4; i++){
    Serial.println(displayLine[i]);
  }
*/  
   //write the result to the OLED display
   myOLED << gloHome;
   for (int i = 0; i<4; i++){
     myOLED << displayLine[i];
   }
  
}




// display current OD, raw light sensor value and blank value on serial LCD
void displayTubeStatus(int tubeNum){
  //Serial << "\n************** displayTubeStatus() ******************* tubeNum=" << tubeNum;

  char bval[15]; //note: if these character arrays are too short, they can overwrite each other
  char odval[15];
  char ltin[15];
  char topLine[] = "Tube:   Blk=    ";
  char botLine[] = "OD=     Raw=    ";
  
  //convert variables to strings of the appropriate length
  dtostrf(blankValue[tubeNum], 4, 0, bval);
  //Serial.println();
  //dtostrf(LEDonReading[lastButtonPressed], 4, 0, ltin); //use LEDonReading for the raw value.  
                                                          //We want to be able to see how close we are to saturating the detector when we're adjusting the potentiometer.
                                                          //Note that the raw value will show fluctuations due to ambient light, but the lightIn value (i.e. abient light subtracted)
                                                          //is what is being used to calculate the OD value
  dtostrf(lightIn[tubeNum], 4, 0, ltin);        //use lightIn for raw value
  //  Serial <<"       ltin=" <<ltin;
  //Serial.println();
                                                          //not sure which I prefer, lightIn or LEDonReading, for raw value
                                                          //using lightIn for now
  dtostrf(ODvalue[tubeNum], 4, 2, odval);
  // Serial <<"       odval=" <<odval;
  //Serial.println();

  //set tube number
  topLine[5] = tubeNum + 49; //convert integer to ascii by adding 48, add one more to shift index by 1
  
  //set blank, OD and raw numbers
  for (int i=0; i < 4; i++){
     topLine[12+i] = bval[i]; //set blank value
     //Serial << "\n\t\t\t\t bval=" <<bval <<" bval[" <<i<< "]=" << bval[i] << "   ^^^ topLine=" << topLine;
     botLine[12+i] = ltin[i]; //set raw value
     botLine[3+i] = odval[i]; //set OD value
  }

  //send OLED display output to serial port
  //Serial << "\ntopLine=" << topLine;
  //Serial << "\nbotLine=" << botLine;
  
  //send OLED display output to OLED display
  myOLED << gloHome << topLine << gloSetCursor(0,1) << botLine; //output compiled lines to OLED display
  myOLED << "                                "; //clear the bottom two lines of the display
}


//send data to Thingspeak
//OD_CHANNEL_ID
//OD_API_KEY
//TEMP_CHANNEL_ID
//TEMP_API_KEY
void uploadDataToThingspeak(){
  //loop through datastreams (except temperature) and update data
  for (int i=0; i<numTubes; i++){
      ThingSpeak.setField(i+1, ODvalue[i]); //set each field one-by-one, thingspeak channels are indexed from 1
    }
  Serial.println("sending OD data to thingspeak");
  Serial.println(ThingSpeak.writeFields(OD_CHANNEL_ID, OD_API_KEY));

  //send temperature data to thingspeak
  Serial.println("sending temperature data to thingspeak");
  ThingSpeak.setField(1, temperature);
  ThingSpeak.writeFields(TEMP_CHANNEL_ID, TEMP_API_KEY);
  
}


void readLightSensors(){
  // clear the lightIn value
  for (int i = 0; i<numTubes; i++){
   lightIn[i] = 0;
   ODvalue[i] = 0;
   //checkBlankButtons(); removed this in version 20 of the program to see if I can fix the software "freezing" bug.
  }
  // start accumulating lightIn values
  for (int j = 0; j<pointsToAverage; j++){ 
    // read light sensor values with the LED off
    // this measures ambient light levels, which will later get subtracted from the reading
    digitalWrite(ledPin, LOW); //turn off LEDs
    delay(10); //it takes ~1ms for the light senor reading to stabilize after the LED has been turned off
    for (int i = 0; i < numTubes; i++){
      LEDoffReading[i] = analogRead(lightInPin[i]);      
    }
    // read light sensor values with the LED on
    digitalWrite(ledPin, HIGH); //turn on LEDs
    delay(10); //it takes ~1ms for the light senor reading to stabilize after the LED has been turned on
    for (int i = 0; i < numTubes; i++){
      LEDonReading[i] = analogRead(lightInPin[i]);      
    }   
    // calculate the difference and add it to lightIn
    for (int i = 0; i < numTubes; i++){
      lightIn[i] += (LEDonReading[i] - LEDoffReading[i]);
    } 
  }
  // divide lightIn by pointsToAverage to get the average value
  for (int i = 0; i < numTubes; i++){
      lightIn[i] = lightIn[i]/pointsToAverage;
      ODvalue[i] = -(log10(lightIn[i]/blankValue[i]));
  }
  // turn the lights off when you're done
  digitalWrite(ledPin, LOW); //turn off LEDs
}


// ******************* TEMPERATURE SENSOR SUBROUTINES **********************************
//initialize DS18B20 temperature sensor
boolean initTemp(){
  Serial.println("^^^initTemp()^^^");
  byte i;
  
 //find a device
 temperatureDeviceFound = ds.search(tempSensAddr);
 
 if (!temperatureDeviceFound) {
   ds.reset_search(); //if no devices are found, reset the search (i.e. begin a new search next time)
   Serial.println("error: no devices found");
   return false; //return -1000 when there is an error
 }
 //display the address of the device that was found
 Serial.print("tempSensAddr=");
  for( i = 0; i < 8; i++) {
    Serial.print(tempSensAddr[i], HEX);
    Serial.print(" ");
  }
 
 // assuming a device was found, do a cyclical redundancy check (CRC)
 if (OneWire::crc8( tempSensAddr, 7) != tempSensAddr[7]) {
   Serial.println("error: CRC check failed");
   return false; //return -1000 when there is an error
 }
 
 if (tempSensAddr[0] != DS18S20_ID && tempSensAddr[0] != DS18B20_ID) {
   Serial.println("error: wrong type of temp sensor IC");
   return false; //return -1000 when there is an error
 }
 
 return true; //if a device was found and there were no errors
}



//start the temperature conversion
//after this command, you need to wait about 1000ms to read the data
// based on suggestions from this website www.cupidcontrols.com/2014/10/moteino-arduino-and-1wire-optimize-your-read-for-speed/
void startTempConversion(){
   //Serial.println("^^^startTempConversion()^^^");
   if (temperatureDeviceFound){
     ds.reset();
     ds.select(tempSensAddr);
     
     // Start conversion
     ds.write(0x44); // start conversion, no parasitic power
   }
}


//returns the temperature from one DS18S20 in DEG Celsius
//only works with one DS18S20
//note, this was rewritten in version 20
float getTemp(){
  //Serial.println("^^^getTemp()^^^");
   if (temperatureDeviceFound){
     float temp;
     byte i;
     byte present = 0;
     byte data[12];

     present = ds.reset();
     ds.select(tempSensAddr);
     
     // Issue Read scratchpad command
     ds.write(0xBE);
     
     // Receive 9 bytes
     for ( i = 0; i < 9; i++) {
     data[i] = ds.read();
     }
     
     // Calculate temperature value
     temp = ( (data[1] << 8) + data[0] )*0.0625;
     ds.reset_search(); //reset the search once you're done reading
     return temp;
   }
   return -1000; //error to return if no temperature device was found
}





