/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
  This example shows how to send HID (keyboard/mouse/etc) data via BLE
  Note that not all devices support BLE keyboard! BLE Keyboard != Bluetooth Keyboard
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
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
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// GPIO corresponding to HID gamepad
#define LEFT_PIN    6
#define RIGHT_PIN   5
#define UP_PIN      10
#define DOWN_PIN    9

#define BUTTON11_PIN 11
#define BUTTON12_PIN 12

#define START_BUTTON_PIN 13

#define pinsize 12

int buttonPins[pinsize]                 = {
                                     LEFT_PIN, RIGHT_PIN, UP_PIN, DOWN_PIN,
                                     A0, A1, A2, A3,
                                     A4, A5, BUTTON11_PIN, BUTTON12_PIN
                                     }; 
int buttonState[pinsize]                = {
                                    HIGH, HIGH, HIGH, HIGH,
                                    HIGH, HIGH, HIGH, HIGH,
                                    HIGH, HIGH, HIGH, HIGH
                                    };

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit HID Gamepad"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Feather Stick': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Feather Joystick" )) ) {
    error(F("Could not set device name?"));
  }

  /* !Deprecatated - Enable HID Service */
  // Enable HID Service if not enabled
  int32_t hid_en = 0;  
  ble.sendCommandWithIntReply( F("AT+BLEHIDEN?"), &hid_en);
  if ( !hid_en )
  {
    Serial.println(F("Enable HID: "));
    ble.sendCommandCheckOK(F( "AT+BLEHIDEN=1" ));
  }

  // Enable HID Keyboard Service if not enabled
  int32_t keyboard_en = 0;
  ble.sendCommandWithIntReply( F("AT+BLEKEYBOARDEN?"), &keyboard_en);
  if ( !keyboard_en ) {
    Serial.println(F("Enable HID Keyboard: "));
    ble.sendCommandCheckOK(F( "AT+BLEKEYBOARDEN=on" ));
  }

  // Enable HID Gamepad Service if not enabled
  int32_t gamepad_en = 0;
  ble.sendCommandWithIntReply( F("AT+BLEHIDGAMEPADEN?"), &gamepad_en);
  if ( !gamepad_en )
  {
    Serial.println(F("Enable HID GamePad: "));
    ble.sendCommandCheckOK(F( "AT+BLEHIDGAMEPADEN=1" ));
  }

  /* Add or remove service requires a reset */
  if ( !hid_en || !keyboard_en || !gamepad_en ) {
    Serial.println(F("Performing a SW reset (service changes require a reset): "));
    if ( !ble.reset() ) {
      error(F("Couldn't reset??"));
    }
  }

  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts gamepad input"));
  Serial.println();

  // Set up input Pins
  for(int i=0; i< pinsize; i++)
  {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  if ( ble.isConnected() )
  {
    for (int pin = 0; pin < pinsize; pin++ ) {
      buttonState[pin] = digitalRead(buttonPins[pin]);
    }

    int x = 0;
    if ( buttonState[0] == LOW )
    {
      x += -1;
    }
    else if ( buttonState[1] == LOW )
    {
      x += 1;
    }

    int y = 0;
    if ( buttonState[2] == LOW )
    {
      y += -1;
    }
    else if ( buttonState[3] == LOW )
    {
      y += 1;
    }

    int buttons = 0;
    for (int j = 4; j < pinsize; j++)
    {
      if ( buttonState[j] == LOW )
      {
        // buttons |= 1 << (j - 4); // --> only available when 4 buttons.
        buttons |= (int)(pow(2, j - 4) + 0.5);
      }
    }

    // Send axises and buttons
    String command = String("AT+BLEHIDGAMEPAD=") + String(x, DEC) + "," + String(y, DEC) + "," + String(buttons);
    ble.println(command);

    int stbtn = digitalRead(START_BUTTON_PIN);
    if (stbtn == 0)
    {
      sendTo32u4("00-00-14-00");
    }

  }
  /*
  for (int pin = 0; pin < 4; pin++)
  {
    int swt = digitalRead(pin + 10);
    if (swt == 0)
    {
      char *bleCode;
      switch (pin)
      {
        case 0: 
          bleCode = "00-00-50-00"; // left
          break;
        case 1: 
          bleCode = "00-00-4F-00"; // rigth
          break;
        case 2:
          bleCode = "00-00-51-00"; // down
          break;
        case 3:
          bleCode = "00-00-52-00"; // up
          break;
      }

      if (strlen(bleCode) > 1)
      {
        sendTo32u4(bleCode);
      }
    }
  }
  */

  /*
  int btn1 = digitalRead(9);
  // Serial.println(btn1);
  if (btn1 == 0)
  {
    sendTo32u4("00-00-14-00");
  }
  */

  // scaning period is 50 ms
  delay(100);
}

void sendTo32u4(char *bleCode)
{
    // ble.print("AT+BleKeyboard=");
    // ble.println(49);
    // https://learn.adafruit.com/introducing-adafruit-ble-bluetooth-low-energy-friend/ble-services
    ble.print("AT+BLEKEYBOARDCODE=");
    ble.println(bleCode);

    ble.print("AT+BLEKEYBOARDCODE=");
    ble.println("00-00");

    if( ble.waitForOK() )
    {
      Serial.println( F("OK!") );
    }else
    {
      Serial.println( F("FAILED!") );
    }
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
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
