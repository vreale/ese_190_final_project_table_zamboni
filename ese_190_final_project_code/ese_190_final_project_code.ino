#define trigPin 13
#define echoPin 4
#include <Adafruit_ATParser.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>

/*********************************************************************
 This example has been modified by Matthew Wolfman and Sheon Mwapinza
 for the purposes of University of Pennsylvania ESE 111
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the software serial bluefruit object

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("ESE 111 Adafruit Bluefruit Lab"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset. This happens pretty freqeuntly the first time you switch computers or change the sketch.\nPlease try re- uploading the code 1 or 2 more times."));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* Print Bluefruit information */
  //ble.info();

  char command[BUFSIZE+1];
  Serial.println(F("************************************************************"));
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("************************************************************"));

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // Check BLE version
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println("\n ********BLE PAIRED SUCCESSFULLY. READY TO GO!!!********\n");
  Serial.println(F("************************************************************"));

pinMode (trigPin , OUTPUT );
pinMode (echoPin , INPUT );
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
const int Mleft = 6; //controll for the left motor
const int Mright = 5; //controll for the right motor

//controls speed of the left motor
const int left_speed = 900;

//controls speed of the right motor
const int right_speed = 900;

////controls turning degree
const int turn_deg = 100;
int sensorValue = 0;        // value read from the pot
int LoutputValue = 0;
int leftOrRight = 0;        //left = 0, right = 1
int start = 0;


void loop(void)
{
  if(!start) {
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  
  /* Wait for new data to arrive */
  
  if (len == 0 && !start) return;

  //Check if a button was pressed
  if (packetbuffer[1] == 'B') {
    
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
//    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      if (buttnum == 1 && !start) {
         start = 1;
      } else if (buttnum == 1) {
        analogWrite(Mleft, 0);   
        analogWrite(Mright, 0); 
        start = 0;
      }
      }
      Serial.begin(9600);
    } else {
//      Serial.println(" released");
    }
}
if(start) {    
long duration , distance;
digitalWrite (trigPin , LOW ); // start trig at 0
delayMicroseconds (2);
digitalWrite (trigPin , HIGH ); //The rising edge of trig pulse
delayMicroseconds (10); // decides duration of trig pulse
digitalWrite (trigPin , LOW ); //falling edge of the trig pulse
// NOTE: echo pin reads HIGH till it receives the reflected signal
duration = pulseIn (echoPin , HIGH ); // Reading the duration for which echoPin was HIGH gives
//you the time the sensor receives a reflected signal at the echo pin
distance = (duration / 2) / 29.1; //Calculate the distance of the reflecting surface in cm
//Serial.begin(9600);
Serial.print("distance:");
Serial.println(distance);

 if(distance > 10) {
  //STOP
  analogWrite(Mleft, 0);   // Left Motor -> LOW
  analogWrite(Mright, 0);  // Right Motor -> LOW
  delay(500);
  if(leftOrRight == 0) {
    leftOrRight = 1;
    analogWrite(Mleft, 0);   // Left Motor -> LOW
    analogWrite(Mright, 100);  // RIGHT Motor -> HIGH (Speed set by Pot)
    delay(2000);
  } else {
    leftOrRight = 0; 
    analogWrite(Mleft, 110);   // Left Motor -> LOW
    analogWrite(Mright, 0);  // RIGHT Motor -> HIGH (Speed set by Pot)
    delay(2000);
  }
 } else {
   analogWrite(Mleft, 80);   // Left Motor -> LOW
   analogWrite(Mright, 80);
 }
}
// delay(2000);
}

/*If you want to send commands through the terminal
 * NB: this requires your Arduino to be constantly 
 * connected to the computer so no wireless!
 */
void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count=0;

  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.available() == 0) );
}


/* Below is code to paste into the main loop if you want to use different app 
 *  functionality for your final project. You do not need it for the lab.
 
 // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
      if (buttnum == 1) {
        digitalWrite(1,HIGH);
        digitalWrite(2,LOW);
        digitalWrite(3,LOW);
      }
      if (buttnum == 2) {
        digitalWrite(1,LOW);
        digitalWrite(2,HIGH);
        digitalWrite(3,LOW);
      }
      if (buttnum == 1) {
        digitalWrite(1,LOW);
        digitalWrite(2,LOW);
        digitalWrite(3,HIGH);
      }
      }
    } else {
      Serial.println(" released");
    }
  
  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red_val = packetbuffer[2];
    uint8_t green_val = packetbuffer[3];
    uint8_t blue_val = packetbuffer[4];
    Serial.println ("Received RGB #");
    if (red_val < 0x10) Serial.print("0");
    Serial.println(red_val);
    if (green_val < 0x10) Serial.print("0");
    Serial.println(green_val);
    if (blue_val < 0x10) Serial.print("0");
    Serial.println(blue_val);
    Serial.println();

    analogWrite(red_pin, red_val);
    analogWrite(green_pin, green_val);
    analogWrite(blue_pin, blue_val);

  
  } 
  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer+2);
    lon = parsefloat(packetbuffer+6);
    alt = parsefloat(packetbuffer+10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Accel\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Mag\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
  */
