#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GPS
#include <MicroNMEA.h>
#include "SparkFun_Ublox_Arduino_Library.h"

#include "sys/time.h"

#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEBeacon.h"
#include "esp_sleep.h"

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>




/********************************** Private Defines **********************/

// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1
#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
// Zoe-M8Q pins
#define I2C_SDA_2            18
#define I2C_SCL_2            19

//////////////////////////////////////////////////////////////////////////

/********************************* Global Variables *********************/

// Your GPRS credentials (leave empty, if not needed)
const char apn[]      = ""; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 

// Server details
// The server variable can be just a domain name or it can have a subdomain. It depends on the service you are using
const char server[] = "example.com"; // domain name: example.com, maker.ifttt.com, etc
const char resource[] = "/post-data.php";         // resource path, for example: /post-data.php
const int  port = 80;                             // server port number

// Keep this API Key value to be compatible with the PHP code provided in the project page. 
// If you change the apiKeyValue value, the PHP file /post-data.php also needs to have the same key 
String apiKeyValue = "tPmAT5Ab3j7F9";

char nmeaBuffer[100];



SFE_UBLOX_GPS myGPS;
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
TinyGsm modem(SerialAT);

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// I2C for zoe-M8Q sensor
TwoWire I2CSensors = TwoWire(1);

// TinyGSM Client for Internet connection
TinyGsmClient gsmClient(modem);


BLEAdvertising *pAdvertising;
struct timeval now;
                              
#define BEACON_UUID           "8ec76ea3-6668-48da-9866-75be8bc86f4d" // UUID 1 128-Bit (may use linux tool uuidgen or random numbers via https://www.uuidgenerator.net/)

esp_bd_addr_t*  ble_addr;
/////////////////////////////////////////////////////////////////////

/****************************** Function Prototypes *****************/
bool setPowerBoostKeepOn(int en);
void setBeacon();

//////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting Application");


  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  I2CSensors.begin(I2C_SDA_2, I2C_SCL_2);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  Serial.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  /*

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  modem.restart();

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
  */
  


  if (myGPS.begin(I2CSensors, 0x42) == false)
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  myGPS.setI2COutput(COM_TYPE_NMEA); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  // Create the BLE Device
  BLEDevice::init("");
  ble_addr = (BLEDevice::getAddress().getNative());

  Serial.printf("String BLE Address: %s", BLEDevice::getAddress().toString().c_str());
  pAdvertising = BLEDevice::getAdvertising();
  
  setBeacon();

  // Start advertising
  pAdvertising->start();
  Serial.println("Advertizing started...");
  delay(1000);
  pAdvertising->stop();
  Serial.println("Advertizing stoped!");

}

void loop()
{
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

  if(nmea.isValid() == true)
  {
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();

    Serial.print("Latitude (deg): ");
    Serial.print(latitude_mdeg / 1000000., 6);
    Serial.print("\tLongitude (deg): ");
    Serial.println(longitude_mdeg / 1000000., 6);
  }
  else
  {
    Serial.print("No Fix - ");
    Serial.print("Num. satellites: ");
    Serial.println(nmea.getNumSatellites());
  }

  // Start advertising
  pAdvertising->start();
  Serial.println("Advertizing started...");
  delay(1000);
  pAdvertising->stop();
  Serial.println("Advertizing stoped!");

  delay(1000); //Don't pound too hard on the I2C bus
}

//This function gets called from the SparkFun Ublox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}


bool setPowerBoostKeepOn(int en) 
{
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

void setBeacon() {

  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  BLEAdvertisementData oScanResponseData = BLEAdvertisementData();
  
  //oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04
  
  std::string strServiceData = "";

  uint8_t adv_data[31];
  uint8_t adv_data_len;

    adv_data[0] = 0xAA;                     // Preamble
    adv_data[1] = 0x8E;                     // ACC Addr byte 1
    adv_data[2] = 0x89;                     // ACC Addr byte 2
    adv_data[3] = 0xBE;                     // ACC Addr byte 3
    adv_data[4] = 0xD6;                     // ACC Addr byte 4
    adv_data[5] = 0x20;                     // PDU HDR byte 1
    adv_data[6] = 0x25;                     // PDU HDR byte 2
    adv_data[7] = *(*ble_addr + 0);         // BLE ADDRESS byte 1
    adv_data[8] = *(*ble_addr + 1);         // BLE ADDRESS byte 2
    adv_data[9] = *(*ble_addr + 2);         // BLE ADDRESS byte 3
    adv_data[10] = *(*ble_addr + 3);        // BLE ADDRESS byte 4
    adv_data[11] = *(*ble_addr + 4);        // BLE ADDRESS byte 5
    adv_data[12] = *(*ble_addr + 5);        // BLE ADDRESS byte 6
    adv_data[13] = 0x1E;                     // AD Flag byte 1
    adv_data[14] = 0xFF;                     // AD Flag byte 2
    adv_data[15] = 0x02;                     // AD Flag byte 3
    adv_data[16] = 0x00;                     // AD Flag byte 4
    adv_data[17] = 0x0D;                     // AD APP
    adv_data[18] = 0x8E;                     // AD counter
    adv_data[19] = 0x65;  // UUID 11
    adv_data[20] = 0x87;  // UUID 12
    adv_data[21] = 0xAA;  // UUID 13
    adv_data[22] = 0xEE;  // UUID 14
    adv_data[23] = 0xEE;  // UUID 15
    adv_data[24] = 0x07;  // UUID 16
    adv_data[25] = 0x00;  // Major 1 Value
    adv_data[26] = 0x20;  // Major 2 Value
    adv_data[27] = 0x21;  // Minor 1 Value
    adv_data[28] = 0x22;  // Minor 2 Value
    adv_data[29] = 0xA0;  // Beacons TX power

    adv_data_len = 30;

  oAdvertisementData.addData(std::string((char*)adv_data, adv_data_len));
  
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->setScanResponseData(oScanResponseData);

}
