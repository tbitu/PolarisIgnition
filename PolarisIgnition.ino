/**************************************************************************/
/*!
    Ignition control & security project
    All glory to http://www.webnology.ch/ for the EEPROM stuff
    Thanks to singlerider for the initial sketch idea

    SDA  = 4
    SCL  = 5
    D0   = 16
    D1   = 5
    D2   = 4
    D3   = 0
    D4   = 2
    D5   = 14
    D6   = 12
    D7   = 13
    D8   = 15
    RX   = 3
    TX   = 1

    Adafruit PIN	Pin Name	Physical Pin
    0	GPA0	21
    1	GPA1	22
    2	GPA2	23
    3	GPA3	24
    4	GPA4	25 
    5	GPA5	26
    6	GPA6	27
    7	GPA7	28
    8	GPB0	1
    9	GPB1	2
    10	GPB2	3
    11	GPB3	4
    12	GPB4	5
    13	GPB5	6
    14	GPB6	7
    15	GPB7	8
*/
/**************************************************************************/

#include <SPI.h>
#include <pins_arduino.h>
//PN532 SPI
#include <PN532_SPI.h>
//#include <PN532_I2C.h>
#include "PN532.h"
//#include "LowPower.h"
#include <EEPROM.h>
#include <PubSubClient.h>

//ESP8266
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include "SoftwareSerial.h"
#include <Adafruit_FONA.h>
#include "Wire.h"
#include "Adafruit_MCP23017.h"

Adafruit_MCP23017 mcp;

//Map to ThingsBoard
const char *token = "xxxxxxx"; // From thingsboard.io device
unsigned long lastPost = 0;
unsigned long delayPost = 10000;

//PN532 SPI
#define PN532_SS (D8)
#define SPI_PINS_HSPI 1
PN532_SPI pn532spi(SPI, PN532_SS);
PN532 nfc(pn532spi);

//WiFi
ESP8266WiFiMulti wifiMulti;
const char *ssid1 = "xxxxx";
const char *password1 = "xxxxx";
const char *ssid2 = "xxxxx";
const char *password2 = "xxxxx";
const char *ssid3 = "xxxxx";
const char *password3 = "xxxxx";
const char *ssid4 = "xxxxx";
const char *password4 = "xxxxx";

//PubSubClient
WiFiClient client;
PubSubClient mqtt(client);
unsigned long lastReconnectAttempt = 0;
unsigned long lastMqtt = 0;
unsigned long delayMqtt = 3000;

//OTA
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

// FONA
const int FONA_PWRKEY = 6;
//#define FONA_RX (5)
//#define FONA_TX (4)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     )
//#define SIMCOM_7000
SoftwareSerial fonaSS = SoftwareSerial(2, 0, false, 256);
char PIN[5] = "xxxx"; // SIM card PIN

//HardwareSerial fonaSS(Serial);

// Wake up pin INTERRUPT
const int wakeUpPin = D0; //3 for Atmega

// Engine running pin
const int runningPin = 1; //Atmega 4

// Dead man's switch relay
const int killPin = 7; //Atmega 2

//Ignition button LED
const int ledPin = 3; //Atmega 6
bool ledOn;

//Starter pin
const int startPin = 5; //Atmega 9

//Various variables
uint32_t sleepTimeAddr = 0;
int sleepTime;
bool authenticated = 0;
bool wake = 0;
bool retryWifi = 1;
bool lteEnabled = 0;
bool running = 0;

//Starter variables
unsigned long intervalStart = 3000; // the time we need to wait
unsigned long startMillis;          // millis() returns an unsigned long.
unsigned long currentMillis;        // millis() returns an unsigned long.
bool countStart = 1;

//---------------------------------------------
// EEPROM
//---------------------------------------------

const uint32_t EEPROMSize = 1023; //BE CAREFULL NOT TO EXCEED EEPROM SIZE OF YOUR ARDUINO
//TO AVOID DAMAGING YOUR EEPROM!!!!!!!
const uint32_t memBase = 4; // Start the storage of RFID from this address

//---------------------------------------------
// RFID MASTER, HARDCODED
//---------------------------------------------
uint32_t uid_master = 0123456789;

uint32_t master_mode = 0;
uint32_t master_mode_counter = 0;

//---------------------------------------------
// DEBUG
//---------------------------------------------

//boolean debug = true;
boolean debug = false;

//FONA
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
char imei[16] = {0}; // Use this for device ID
uint8_t type;
uint16_t battLevel = 0; // Battery level (percentage)
float latitude, longitude, speed_kph, heading, altitude, second;
uint16_t year;
uint8_t month, day, hour, minute;

char URL[200];  // Make sure this is long enough for your request URL
char body[100]; // Make sure this is long enough for POST body
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
    headBuff[12], altBuff[12], tempBuff[12], battBuff[12];

// The following line is used for applications that require repeated data posting, like GPS trackers
#define samplingRate 60 // The time in between posts, in seconds

//Uptime
unsigned long previousTime = 0;
byte seconds = 0;
byte minutes = 0;
byte hours = 0;

char *uptime(unsigned long milli)
{

  static char _return[32];
  unsigned long secs = milli / 1000, mins = secs / 60;
  unsigned int hours = mins / 60, days = hours / 24;
  milli -= secs * 1000;
  secs -= mins * 60;
  mins -= hours * 60;
  hours -= days * 24;
  //sprintf(_return, "Uptime %d days %2.2d:%2.2d:%2.2d.%3.3d", (byte)days, (byte)hours, (byte)mins, (byte)secs, (int)milli);
  sprintf(_return, "%2.2d:%2.2d:%2.2d", (byte)hours, (byte)mins, (byte)secs, (int)milli);
  return _return;
}
char *uptime()
{ // Function made to millis() be an optional parameter

  return (char *)uptime(millis()); // call original uptime function with unsigned long millis() value
}

// Handler for the pin interrupt.
void wakeUp()
{
}

//---------------------------------------------
// send mqtt
//---------------------------------------------

boolean reconnect()
{
  if (mqtt.connect("PTMQTT", "username", "password"))
  {
    // Once connected, publish an announcement...
    // mqtt.publish("PTMQTT/wake", "Hello, reconnected!");
    // ... and resubscribe
    mqtt.subscribe("PTMQTT/changeEngine");
    mqtt.subscribe("PTMQTT/changeSecurity");
    mqtt.subscribe("PTMQTT/changeSleepTime");
    mqtt.subscribe("PTMQTT/cmd");
    mqtt.subscribe("PTMQTT/wake");
  }
  return mqtt.connected();
}
void callback(char *topic, byte *payload, unsigned int length)
{
  if (debug)
  {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++)
    {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  }
  if (strcmp(topic, "PTMQTT/changeSecurity") == 0)
  {
    if (debug)
      Serial.println("Changing Security status by MQTT");
    if ((char)payload[0] == '1')
    {
      // Authentication accepted
      authenticated = 1;
      mcp.digitalWrite(killPin, LOW);
      mqtt.publish("PTMQTT/Security", String(authenticated).c_str());
      if (debug)
        Serial.println("Security disabled by MQTT");
    }
    else if ((char)payload[0] == '0')
    {
      // Authentication off
      authenticated = 0;
      mcp.digitalWrite(killPin, HIGH);
      mqtt.publish("PTMQTT/Security", String(authenticated).c_str());
      if (debug)
        Serial.println("Security enabled by MQTT");
    }
  }
  if (strcmp(topic, "PTMQTT/changeEngine") == 0)
  {
    if (debug)
      Serial.println("Changing Engine status by MQTT");
    if ((char)payload[0] == '1')
    {
      // Engine on and authenticated
      if (mcp.digitalRead(runningPin) == LOW && authenticated)
      {
        if (debug)
          Serial.println("Starting by MQTT");
        mcp.digitalWrite(startPin, LOW);
        if (countStart)
        {
          if (debug)
            Serial.println("Starting - 4 seconds by MQTT");
          startMillis = millis();
          countStart = 0;
        }
      }
    }
    else if ((char)payload[0] == '0')
    {
      // Engine off
      authenticated = 0;
      mcp.digitalWrite(killPin, HIGH);
      if (debug)
        Serial.println("Security enabled by MQTT");
    }
  }
  if (strcmp(topic, "PTMQTT/changeSleepTime") == 0)
  {
    payload[length] = '\0';
    String s = String((char *)payload);
    sleepTime = s.toInt();
    if (debug)
    {
      Serial.print("Changing sleepTime to ");
      Serial.println(sleepTime);
    }
    EEPROM.put(sleepTimeAddr, sleepTime);
    //Only for ESP8266!
    EEPROM.commit();
    //
  }
  if (strcmp(topic, "PTMQTT/wake") == 0)
  {
    if ((char)payload[0] == '1')
    {
      if (debug)
        Serial.println("Awake!");
      if (digitalRead(wakeUpPin) == HIGH)
        wake = 1;
    }
    else if ((char)payload[0] == '0')
    {
      if (debug)
        Serial.println("Not being woken");
      wake = 0;
    }
    retryWifi = 0;
  }
  if (strcmp(topic, "PTMQTT/cmd") == 0)
  {
    if ((char)payload[0] == '1')
    {
      if (debug)
        Serial.print("Restarting!");
      ESP.restart();
    }
    else if ((char)payload[0] == 'F')
    {
      // Initialize EEPROM to blank
      initializeEeprom();
      if (debug)
        Serial.println("Initializing EEPROM");
    }
    else if ((char)payload[0] == 'D')
    {
      // Turn on debugging
      debug = true;
      Serial.begin(115200);
      Serial.println("Hello!");
      if (debug)
        Serial.println("Debugging on!");
    }
  }
}

// Power on the module
void powerOn()
{
  mcp.digitalWrite(FONA_PWRKEY, HIGH);
}

void powerOff()
{
  mcp.digitalWrite(FONA_PWRKEY, LOW);
}

void moduleSetup()
{
  fonaSS.begin(9600); // Default LTE shield baud rate
  //delay(100);
  if (!fona.begin(fonaSS)) // Don't use if statement because an OK reply could be sent incorrectly at 115200 baud
    Serial.println(F("Error initializing serial to LTE"));
  else if (debug)
  {
    type = fona.type();
    Serial.println(F("FONA is OK"));
    Serial.print(F("Found "));
    switch (type)
    {
    case SIM7000A:
      Serial.println(F("SIM7000A (American)"));
      break;
    case SIM7000C:
      Serial.println(F("SIM7000C (Chinese)"));
      break;
    case SIM7000E:
      Serial.println(F("SIM7000E (European)"));
      break;
    case SIM7000G:
      Serial.println(F("SIM7000G (Global)"));
      break;
    case SIM7500A:
      Serial.println(F("SIM7500A (American)"));
      break;
    case SIM7500E:
      Serial.println(F("SIM7500E (European)"));
      break;
    default:
      Serial.println(F("???"));
      break;
    }
  }

  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (debug)
  {
    if (imeiLen > 0)
    {
      if (debug)
      {
        Serial.print("Module IMEI: ");
        Serial.println(imei);
      }
    }
  }
}

bool netStatus()
{
  int n = fona.getNetworkStatus();
  if (debug)
  {
    Serial.print(F("Network status "));
    Serial.print(n);
    Serial.print(": ");
    if (n == 0)
      Serial.println(F("Not registered"));
    if (n == 1)
      Serial.println(F("Registered (home)"));
    if (n == 2)
      Serial.println(F("Not registered (searching)"));
    if (n == 3)
      Serial.println(F("Denied"));
    if (n == 4)
      Serial.println(F("Unknown"));
    if (n == 5)
      Serial.println(F("Registered roaming"));
  }
  if (!(n == 1 || n == 5))
    return false;
  else
    return true;
}

//---------------------------------------------
// set LEDs
//---------------------------------------------
void led(uint8_t status = 1)
{
  switch (status)
  {
  case 1:
    // ready to read RFID   blink
    if (!ledOn)
    {
      mcp.digitalWrite(ledPin, HIGH);
      ledOn = 1;
    }
    else
    {
      mcp.digitalWrite(ledPin, LOW);
      ledOn = 0;
    }
    delay(100);
    break;
  case 2:
    // RFID authorized    solid
    mcp.digitalWrite(ledPin, HIGH);
    delay(150);
    break;
  case 3:
    // RFID read MASTER    slow blink
    // inclusion/exclusion MODE
    mcp.digitalWrite(ledPin, HIGH);
    delay(300);
    mcp.digitalWrite(ledPin, LOW);
    delay(300);
    break;
  case 4:
    // RFID included     long on blink
    for (int count = 1; count < 8; count++)
    {
      mcp.digitalWrite(ledPin, HIGH);
      delay(100);
      mcp.digitalWrite(ledPin, LOW);
      delay(20);
    }
    break;
  case 5:
    // RFID excluded      long off blink
    for (int count = 1; count < 8; count++)
    {
      mcp.digitalWrite(ledPin, HIGH);
      delay(20);
      mcp.digitalWrite(ledPin, LOW);
      delay(100);
    }
    break;
  case 6:
    // led off
    mcp.digitalWrite(ledPin, LOW);
    break;
  }
}

//---------------------------------------------
//get RFID as int
//---------------------------------------------
uint64_t getCardIdAsInt(uint8_t uid[], uint8_t uidLength)
{

  if (uidLength == 4)
  {
    // We probably have a Mifare Classic card ...
    uint32_t cardid = uid[0];
    cardid <<= 8;
    cardid |= uid[1];
    cardid <<= 8;
    cardid |= uid[2];
    cardid <<= 8;
    cardid |= uid[3];
    if (debug)
    {
      Serial.print("Seems to be a Mifare Classic card #");
      Serial.println(cardid);
    }
    return cardid;
  }

  else if (uidLength == 7)
  {
    // We probably have a Mifare Ultralight card ...
    uint64_t cardid = 0;
    memcpy(&cardid, uid, sizeof(uid));

    if (debug)
    {
      Serial.println("Seems to be a Mifare Ultralight card #");
      // Print function does not support 64 bit
    }
    if (debug)
    {
      for (uint8_t i = 0; i < uidLength; i++)
      {
        Serial.print(" byte ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.print(uid[i], HEX);
        Serial.println(" ");
      }
    }
    return cardid;
  }
}

//-------------------------------------------------------------------------------------------------------
// EEPROM Functions
//-------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------
// erase EEPROM
//---------------------------------------------------------------
void initializeEeprom()
{
  Serial.println("------------------------------------------------------");
  Serial.println("Initializing EEPROM by erasing all RFIDs              ");
  Serial.println("Setting values of EEPROM addresses to 0               ");
  Serial.println("EEPROM max memory size:                               ");
  Serial.println(EEPROMSize);
  Serial.println("------------------------------------------------------");

  byte zero = 0;

  for (uint32_t adr = memBase; adr <= EEPROMSize; adr++)
  {
    EEPROM.write(adr, zero);
  }
}

//---------------------------------------------------------------
// printEeprom()
// Print the EEPROM to serial output
// for debugging
//---------------------------------------------------------------

void printEeprom()
{
  uint32_t ads = memBase;
  while (ads <= EEPROMSize)
  {
    byte output = EEPROM.read(ads);
    if ((ads % 10) == 0)
      Serial.println(" ");
    Serial.print(ads);
    Serial.print(" => ");
    Serial.print(output, HEX);
    Serial.print("   ");
    ads++;
  }
  Serial.println(" ");
}

//---------------------------------------------------------------
// findRfidInEeprom(uidLength, uid)
// looks for matching RFID
// returns the address of the first byte of RFID (length indication) if found
// returns -1 if NOT found
// returns -2 if error
//---------------------------------------------------------------

int32_t findRfidInEeprom(uint8_t uidLength, uint8_t uid[])
{
  byte key = memBase;
  byte val = EEPROM.read(key);
  boolean match = false;

  if (val == 0 && key == 0)
  {
    if (debug)
      Serial.println("EEPROM is empty ");
  }
  else
  {
    while (val != 0)
    {
      if (key >= EEPROMSize)
      {
        if (debug)
          Serial.println("ERROR: EEPROM STORAGE MESSED UP! Return -2"); //this should not happen! If so initialize EEPROM
        return -2;
      }
      if (val == uidLength)
      {
        // check if uid match the uid in EEPROM
        byte uidAddress = key + 1;
        match = true;
        //compare uid bytes
        for (byte i = 0; i < uidLength; i++)
        {
          byte uidVal = EEPROM.read((uidAddress + i));
          //the first byte of uidVal is the next address
          if (uidVal != uid[i])
          {
            //got to next key
            match = false;
            // in case no break => all bytes same
            break;
          }
        }

        if (match)
        {
          if (debug)
          {
            Serial.println("RFID uid matching in Address = ");
            Serial.println(key);
          }
          return key;
        }
      }

      key = key + val + 1;
      val = EEPROM.read(key);
    }
  }
  if (debug)
  {
    Serial.println("No RFID match in EEPROM, returning -1");
  }
  return -1;
}

//---------------------------------------------------------------
// deleteRfidfromEeprom(address, uidLength)
// delete the RFID from EEPROM
// sets the UID values to 0
// in the EEPROM structure there will be a "hole" with zeroes
// we are not shifting the addresses to avoid unnecessary writes
// to the EEPROM. The 'hole' will be filled with next RFID
// storage that has the same uidLength
//
//
//  BUG if rfid was last rfid in eeprom, length is not set to 0 => not needed because by formatting eeprom all is set to 0
//---------------------------------------------------------------
void deleteRfidfromEeprom(uint32_t address, uint8_t uidLength)
{
  byte zero = 0;
  if (debug)
    Serial.println("Erasing RFID");
  for (uint8_t m = 1; m <= uidLength; m++)
  {
    uint32_t adr = address + m;
    if (debug)
    {
      Serial.print("Address: ");
      Serial.print(adr);
      Serial.println(" ");
    }
    EEPROM.write(adr, zero);
  }
  if (debug)
  {
    printEeprom();
  }
}

//---------------------------------------------------------------
// getEndOfRfidsChainInEeprom(uidLength)
// returns the address of the end of the Rfids chain stored in the EEPROM
// returns -1 if no space left
// returns -2 if unexpected error
//---------------------------------------------------------------
int32_t getEndOfRfidsChainInEeprom(uint8_t uidLength)
{
  byte key = memBase;
  byte val = EEPROM.read(key);

  if (val == 0 && key == 0)
  {
    if (debug)
      Serial.println("EEPROM is empty ");
    return key;
  }
  else
  {
    // if length byte indicator is 0 it means it is the end of the RFIDs stored, last RFID stored
    while (val != 0)
    {
      //this should not happen! If so initialize EEPROM
      if (key > EEPROMSize)
      {
        Serial.println("ERROR: EEPROM STORAGE MESSED UP! EXITING STORAGE READ");
        return -2;
      }
      key = key + val + 1;
      val = EEPROM.read(key);
    }
    if ((key + uidLength) > EEPROMSize)
    {
      Serial.println("Not enough space left in EEPROM to store RFID"); //the RFID to be appended at the end of the storage chain exeeds the EEPROM length
      return -1;
    }
    else
      return key;
  }
}

//---------------------------------------------------------------
// getFreeEepromStorageFragment(uint8_t uidLength)
// return the address where to store the RFID
// with the rfidLength specified.
// Instead of just appending the RFID to the end of
// the storage we look for an erased RFID space and
// fill this
// return address
// return -2 if error
// return -1 if no free storage address found
//---------------------------------------------------------------
int32_t getFreeEepromStorageFragment(uint8_t uidLength)
{
  uint32_t key = memBase;
  byte val = EEPROM.read(key); //holds the uidLength stored in EEPROM
  boolean free = false;

  if (val == 0 && key == 0)
  {
    // EEPROM empty, use the address at key = memBase
    return key;
  }
  else
  {
    //loop till the end of storage chain indicated by a zero value in the key position
    while (val != 0)
    {
      //this should not happen! If so initialize EEPROM
      if (key > EEPROMSize)
      {
        Serial.println("ERROR: EEPROM STORAGE MESSED UP! EXITING STORAGE READ");
        return -2;
      }
      // check if uidLength  match the uidLength in EEPROM
      if (val == uidLength)
      {
        uint32_t uidAddress = key + 1;
        free = true;
        //check if uid bytes are all zero => free storage fragment
        for (uint8_t i = 0; i < uidLength; i++)
        {
          byte uidVal = EEPROM.read((uidAddress + i));
          if (uidVal != 0)
          {
            //got to next key
            free = false;
            break;
          }
        }
        // in case no break => all bytes have zero value => free fragment
        if (free)
        {
          return key;
        }
      }
      key = key + val + 1;
      val = EEPROM.read(key);
    }
    return -1;
  }
}

//---------------------------------------------------------------
// getEepromStorageAddress(uint8_t uidLength)
// combination of getFreeEepromStorageFragment
// and getEndOfRfidsChainInEeprom
// return address
// return -1 if no free storage address found
// return -2 if error
//---------------------------------------------------------------
int32_t getEepromStorageAddress(uint8_t uidLength)
{
  int32_t fragment = getFreeEepromStorageFragment(uidLength);
  if (debug)
  {
    Serial.print("getFreeEepromStorageFragment returned ");
    Serial.print(fragment);
    Serial.println(" ");
  }
  // free fragment found
  if (fragment >= 0)
  {
    return fragment;
  }
  // error returned
  else if (fragment == -2)
  {
    return fragment;
  }
  // no free fragment available
  // check if space available at end of rfid storage chain
  else if (fragment == -1)
  {
    int32_t append = getEndOfRfidsChainInEeprom(uidLength);
    if (debug)
    {
      Serial.print("getEndOfRfidsChainInEeprom returned ");
      Serial.print(append);
      Serial.println(" ");
    }
    return append;
  }
  // should never occur, return error
  else
  {
    return -2;
  }
}

//---------------------------------------------------------------
// writeRfidToEeprom(addrees,uidlength,uid)
// write RFID to EEPROM
//---------------------------------------------------------------
void writeRfidToEeprom(uint32_t StoragePositionAddress, uint8_t uidLength, uint8_t uid[])
{
  // Writing into first free address the length of the RFID uid
  EEPROM.write(StoragePositionAddress, uidLength);
  // Writing into the following addresses the RFID uid values (byte per byte)
  uint32_t uidBytePosition = StoragePositionAddress + 1; //next position after addressByte which contains the uidLength
  for (uint8_t r = 0; r < uidLength; r++)
  {
    EEPROM.write(uidBytePosition, uid[r]);
    uidBytePosition++;
  }
  if (debug)
  {
    printEeprom();
  }
}

void ptpublish(void)
{
  mqtt.publish("PTMQTT/uptime", uptime());
  mqtt.publish("PTMQTT/Engine", String(mcp.digitalRead(runningPin)).c_str());
  mqtt.publish("PTMQTT/Security", String(authenticated).c_str());
  mqtt.publish("PTMQTT/sleepTime", String(sleepTime).c_str());
  if (digitalRead(wakeUpPin) == LOW) // Reset wakeup if ignition is on
  {
    wake = 0;
    mqtt.publish("PTMQTT/wake", String(wake).c_str(), true);
  }
}

// SETUP

void setup(void)
{
  pinMode(wakeUpPin, INPUT);

  if (debug)
  {
    Serial.begin(115200);
    Serial.println("Hello!");
  }

  mcp.begin();
  mcp.digitalWrite(startPin, HIGH);
  mcp.pinMode(runningPin, INPUT);
  mcp.pinMode(killPin, OUTPUT);
  mcp.pinMode(ledPin, OUTPUT);
  mcp.pinMode(FONA_PWRKEY, OUTPUT);
  mcp.pinMode(startPin, OUTPUT);
  mcp.digitalWrite(startPin, HIGH);

  //  Neccessary for SoftwareSerial to work!!!
  pinMode(D3, OUTPUT);
  digitalWrite(D3, LOW);

  //WiFi & HTTP stuff
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(ssid1, password1);
  wifiMulti.addAP(ssid2, password2);
  wifiMulti.addAP(ssid3, password3);
  wifiMulti.addAP(ssid4, password4);
  // WiFi.begin(ssid, password);
  httpUpdater.setup(&httpServer);
  httpServer.begin();

  //Only for ESP8266!
  EEPROM.begin(EEPROMSize);
  //

  /********************************************************
    // Befor you run this sketch the first time
    // uncoment the following initializeEeprom();
    // to clear the EEPROM neccessary to be shure
    // that all EEPROM values used for the RFID
    // storage are initialized with 0.
    // Connect to the serial to check if all values are 0,
    // then comment the functions again and start using
    // the Rfid access control system
    /********************************************************/
  //initializeEeprom();
  //EEPROM.commit();
  //delay(10000);
  if (debug)
  {
    printEeprom();
  }

  EEPROM.get(sleepTimeAddr, sleepTime);

  //NFC PN532
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata)
  {
    if (debug)
      Serial.print("Didn't find PN53x board");
    led(2);
    //    while (1)
    //      delay(100); // halt
  }

  // Got ok data, print it out!
  if (debug)
  {
    Serial.print("Found chip PN5");
    Serial.println((versiondata >> 24) & 0xFF, HEX);
    Serial.print("Firmware ver. ");
    Serial.print((versiondata >> 16) & 0xFF, DEC);
    Serial.print(".");
    Serial.println((versiondata >> 8) & 0xFF, DEC);
  }

  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0x01);

  // configure board to read RFID tags
  nfc.SAMConfig();

  //PubSubClient
  mqtt.setServer("serverURL", port);
  mqtt.setCallback(callback);

  if (debug)
    Serial.println("Waiting for an ISO14443A card");

  // Keep trying for WiFi 10 seconds
  while (retryWifi && (millis() < 10000) && (digitalRead(wakeUpPin) == HIGH))
  {
    if (wifiMulti.run() == WL_CONNECTED)
    {
      if (!mqtt.connected())
      {
        unsigned long now = millis();
        if (now - lastReconnectAttempt > 2000)
        {
          lastReconnectAttempt = now;
          // Attempt to reconnect
          if (reconnect())
          {
            lastReconnectAttempt = 0;
          }
        }
      }
      else
      {
        // Client connected
        mqtt.loop();
      }
    }
    delay(100);
  }

  if (mcp.digitalRead(runningPin) == HIGH)
  {
    mcp.digitalWrite(killPin, LOW);
  }
}

/*
   Looping
*/

void loop(void)
{
  // Allow wake up pin to trigger interrupt on low.
  //  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, FALLING);
  httpServer.handleClient();

  //NFC
  boolean success = false;
  uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
  uint8_t uidLength;                     // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  uint64_t cardid;                       // UID as int

  // Go to sleep if ignition is off
  if (!wake && (digitalRead(wakeUpPin) == HIGH))
  //if ((mcp.digitalRead(runningPin) == LOW) && !wake && (digitalRead(wakeUpPin) == HIGH))
  {
    mcp.digitalWrite(killPin, HIGH);  //Save relay power
    mcp.digitalWrite(startPin, HIGH); //Save relay power
    led(6);                           //Save relay power
    authenticated = 0;                //RFID read from before no longer valid
    nfc.shutDown();
    powerOff();
    //fona.powerDown(); // No retries
    //int counter = 0;
    //while (counter < 3 && !fona.powerDown())
    //{ // Try shutting down
    //  Serial.println(F("Failed to power down FONA!"));
    //  counter++; // Increment counter
    //  delay(1000);
    //}
    if (debug)
      Serial.println("Inside powerdown loop");
    delay(10);

    // For ESP sleep
    if (sleepTime * 1000000 > ESP.deepSleepMax())
    {
      ESP.deepSleep(ESP.deepSleepMax()); //Sleep mode for ESP8266
    }
    else
    {
      ESP.deepSleep(sleepTime * 1000000); //Sleep mode for ESP8266
    }
    delay(1);

    //For Atmega sleep
    //    for (int i = 0; i <= sleepTime / 10; i++) {
    //      if (digitalRead(wakeUpPin) == HIGH) {
    //        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); //Sleep mode for AtMega only
    //      }
    //    }
    //    detachInterrupt(digitalPinToInterrupt(wakeUpPin));
  }
  //Turn on kill switch if ignition is on but not yet authenticated

  if (!authenticated)
  {
    if (mcp.digitalRead(runningPin) == LOW)
    {
      mcp.digitalWrite(killPin, HIGH);
    }
  }

  //If started, make sure starter is off
  if (mcp.digitalRead(startPin) == LOW)
  {
    if (mcp.digitalRead(runningPin) == HIGH)
    {
      if (debug)
        Serial.println("Running, disable starter");
      mcp.digitalWrite(startPin, HIGH);
      countStart = 1;
      mqtt.publish("PTMQTT/Engine", String(mcp.digitalRead(runningPin)).c_str());
    }
  }

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)

  if (digitalRead(wakeUpPin) == LOW)
  {
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
    nfc.shutDown();
    wake = 0;
  }

  if (master_mode == 1)
  {
    //stay in master mode for max 10 seconds
    master_mode_counter++;
    if (master_mode_counter >= 30)
    { //reset master mode after 10 sec no RFID was inserted
      master_mode = 0;
      master_mode_counter = 0;
    }
    if (debug)
    {
      Serial.println("MASTER MODE");
    }
    led(3);
  }

  // Found card
  if (success)
  {
    if (debug)
    {
      Serial.println("Found a card!");
      Serial.print("UID Length: ");
      Serial.print(uidLength, DEC);
      Serial.println(" bytes");
      Serial.print("  UID Value: ");
      nfc.PrintHex(uid, uidLength);
    }

    cardid = getCardIdAsInt(uid, uidLength);
    //is MASTER
    if (cardid == uid_master)
    {
      master_mode = 1;
      master_mode_counter = 0;
      if (debug)
        Serial.println("MASTER detected, going into MASTER MODE  ");
      led(3);
    }
    // is not MASTER
    else
    {
      int32_t findUid = findRfidInEeprom(uidLength, uid);

      // is MASTER MODE, include or exclude RFID from storage
      if (master_mode == 1)
      {
        // card rfid already exists so exlude it from storage
        if (findUid != -1)
        {
          if (debug)
          {
            Serial.println("removing card from eeprom");
            Serial.println(" ");
          }
          deleteRfidfromEeprom(findUid, uidLength);
          led(5);
        }
        // card rfid not found in storage so include it
        else if (findUid == -1)
        {
          // check if space to store rfid available
          int32_t storageAddress = getEepromStorageAddress(uidLength);

          // storage available
          if (storageAddress >= 0)
          {
            if (debug)
            {
              Serial.print("storing card in position = ");
              Serial.print(storageAddress);
              Serial.println(" ");
            }
            // storing card
            writeRfidToEeprom(storageAddress, uidLength, uid);
            led(4);
          }
          else
          { // no storage space available or error
            if (debug)
              Serial.println("memory full or error");
          }
        }
        //Only for ESP8266!
        EEPROM.commit();
        master_mode = 0;
      }
      // no MASTER MODE, authorise or deny door access
      else
      {
        // card authorised
        if (findUid != -1)
        {
          // open door
          if (debug)
            Serial.println("Card authorised");
          authenticated = 1;
          led(2);
          mcp.digitalWrite(killPin, LOW);
          //Crank the starter but ONLY for 4 seconds or as long as the card is present unless already running
          if (mcp.digitalRead(runningPin) == LOW)
          {
            if (debug)
              Serial.println("Starting");
            mcp.digitalWrite(startPin, LOW);
            if (countStart)
            {
              if (debug)
                Serial.println("Starting - 4 seconds");
              startMillis = millis();
              countStart = 0;
            }
          }
        }
        else
        {
          // deny access
          if (debug)
            Serial.println("Card not authorised, access denied");
          led(5);
        }
      }
    }
  }
  else
  {
    // PN532 probably timed out waiting for a card
    //if(debug) Serial.println("Timed out waiting for a card");
    if (!authenticated)
      led(1);
    if (authenticated)
      led(2);
    // Turn off cranking the engine after 4 seconds if no card is still present
    if (mcp.digitalRead(startPin) == LOW)
    {
      currentMillis = millis();
      if (currentMillis - startMillis >= intervalStart)
      {
        if (debug)
          Serial.println("Cranking time over and no card detected");
        mcp.digitalWrite(startPin, HIGH);
        countStart = 1;
      }
    }
  }

  if (wifiMulti.run() == WL_CONNECTED)
  {
    if (!mqtt.connected())
    {
      unsigned long now = millis();
      if (now - lastReconnectAttempt > 5000)
      {
        lastReconnectAttempt = now;
        // Attempt to reconnect
        if (reconnect())
        {
          lastReconnectAttempt = 0;
        }
      }
    }
    else
    {
      // Client connected
      unsigned long nowMqtt = millis();
      if (nowMqtt - lastMqtt > delayMqtt)
      {
        lastMqtt = nowMqtt;
        ptpublish();
      }
      mqtt.loop();
    }
  }

  if ((mcp.digitalRead(runningPin) == HIGH) || wake)
  {
    if (!lteEnabled)
    {
      powerOn();
      moduleSetup();

      if (!fona.enableGPS(true))
        if (debug)
          Serial.println(F("Failed to turn on GPS"));

      if (!fona.unlockSIM(PIN))
      {
        if (debug)
          Serial.println(F("Failed to unlock SIM"));
      }
      else
      {
        if (debug)
          Serial.println(F("Sim unlocked!"));
      }

      if (netStatus())
      {
        if (!fona.enableGPRS(true))
        {
          fona.enableGPRS(false);
          fona.enableGPRS(true);
        }
      }
      lteEnabled = true;
    }
    unsigned long nowPost = millis();
    if (nowPost - lastPost > delayPost)
    {
      lastPost = nowPost;
      if (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude, &year, &month, &day, &hour, &minute, &second))
      {
        //while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
        if (debug)
          Serial.println(F("Failed to get GPS location, retrying..."));
        mqtt.publish("PTMQTT/log", "Failed to get GPS location, retrying...");
      }
      else
      {
        if (debug)
        {
          Serial.println(F("Found 'eeeeem!"));
          Serial.println(F("---------------------"));
          Serial.print(F("Latitude: "));
          Serial.println(latitude, 6);
          Serial.print(F("Longitude: "));
          Serial.println(longitude, 6);
          Serial.print(F("Speed: "));
          Serial.println(speed_kph);
          Serial.print(F("Heading: "));
          Serial.println(heading);
          Serial.print(F("Altitude: "));
          Serial.println(altitude);

          // Uncomment this if you care about parsing UTC time
          Serial.print(F("Year: "));
          Serial.println(year);
          Serial.print(F("Month: "));
          Serial.println(month);
          Serial.print(F("Day: "));
          Serial.println(day);
          Serial.print(F("Hour: "));
          Serial.println(hour);
          Serial.print(F("Minute: "));
          Serial.println(minute);
          Serial.print(F("Second: "));
          Serial.println(second);

          Serial.println(F("---------------------"));
        }
        // Post something like temperature and battery level to the web API
        // Construct URL and post the data to the web API

        // Format the floating point numbers
        dtostrf(latitude, 1, 6, latBuff);
        dtostrf(longitude, 1, 6, longBuff);
        dtostrf(speed_kph, 1, 0, speedBuff);
        dtostrf(heading, 1, 0, headBuff);
        dtostrf(altitude, 1, 1, altBuff);
        //  dtostrf(temperature, 1, 2, tempBuff); // float_val, min_width, digits_after_decimal, char_buffer
        //dtostrf(battLevel, 1, 0, battBuff);

        // Also construct a combined, comma-separated location array
        // (many platforms require this for dashboards, like Adafruit IO):
        //      sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff); // This could look like "10,33.123456,-85.123456,120.5"

        // Construct the appropriate URL's and body, depending on request type
        // In this example we use the IMEI as device ID

        // Let's try a POST request to thingsboard.io
        sprintf(URL, "http://demo.thingsboard.io/api/v1/%s/telemetry", token);
        //sprintf(body, "{\"latitude\":%s,\"longitude\":%s,\"speed\":%s,\"head\":%s,\"alt\":%s,\"temp\":%s,\"batt\":%s}", latBuff, longBuff, speedBuff, headBuff, altBuff, tempBuff, battBuff);
        sprintf(body, "{\"lat\":%s,\"long\":%s,\"speed\":%s,\"head\":%s,\"alt\":%s}", latBuff, longBuff, speedBuff, headBuff, altBuff); // If all you want is lat/long

        mqtt.publish("PTMQTT/GPS", body, true);

        if (!fona.postData("POST", URL, body))
        {
          if (netStatus())
          {
            if (!fona.enableGPRS(true))
            {
              fona.enableGPRS(false);
              fona.enableGPRS(true);
            }
            fona.postData("POST", URL, body);
          }
          if (debug)
            Serial.println(F("Failed to complete HTTP POST, retried once"));
        }
      }
    }
  }
}
