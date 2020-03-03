#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GPS
#include <MicroNMEA.h>
#include "SparkFun_Ublox_Arduino_Library.h"

#include "sys/time.h"

#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEBeacon.h"
#include "esp_sleep.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <opendroneid.h>


// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#define USE_SIMULATOR     0
#define USE_SERIAL        1


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
const char server[] = "test.adpmdrones.com"; // domain name: example.com, maker.ifttt.com, etc
const char resource[] = "/add.php";         // resource path, for example: /post-data.php
const int  port = 80;                             // server port number

// Keep this API Key value to be compatible with the PHP code provided in the project page. 
// If you change the apiKeyValue value, the PHP file /post-data.php also needs to have the same key 
String apiKeyValue = "tPmAT5Ab3j7F9";

char nmeaBuffer[100];
char nmeaBufferforPrint[100];
uint8_t bufferIdx = 0;

SFE_UBLOX_GPS myGPS;
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
TinyGsm modem(SerialAT);

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

#if USE_SERIAL  

#define RX_PIN      14
#define TX_PIN      12
#define SerialSensor  Serial2        

#else
// I2C for zoe-M8Q sensor
TwoWire I2CSensors = TwoWire(1);
#endif
// TinyGSM Client for Internet connection
TinyGsmClient gsmClient(modem);


BLEAdvertising *pAdvertising;

                              

esp_bd_addr_t*  ble_addr;

uint8_t timerCounter = 0;
typedef struct {
  double latitude;
  double longitude; // In millionths of a degree
	float altitudeMSL; // In millimetre
  float altitudeWGS84;
	float speed;
  float course;
}GPS_Data_t;
GPS_Data_t gps_data;
ODID_Location_encoded beaconDataEncoded;
ODID_Location_data beaconData;

ODID_BasicID_encoded basicID_enc;
ODID_BasicID_data basicID;


hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile bool updateBasicID = false;
volatile uint32_t lastIsrAt = 0;
char id[] = "ABCDF1FG916003A12345";
// Timers auxiliar variables
long now = millis();
long lastMeasure = 0;

/////////////////////////////////////////////////////////////////////

/****************************** Function Prototypes *****************/
bool SetPowerBoostKeepOn(int en);
void SetBeacon();
float GetTimeStampData();
void PreparePacketData(ODID_Location_data * inData, GPS_Data_t* p_gps_data);
void setBeaconLocationData(ODID_Location_encoded *PEncodedLocation);
void setBeaconIDData(ODID_BasicID_encoded* pBasicID_enc);
void UpdateDataBase(void);


void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  if(isrCounter == 10) { // 300ms * 10 = 3000ms = 3sec
    isrCounter = 0;
    updateBasicID = true;
  }
  else {
    updateBasicID = false;
    isrCounter++;
  }
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}


//////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting Application");


  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  #if USE_SIMULATOR
    Serial.println("Using Simulator!");
    SerialSensor.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  #else
  
  #if USE_SERIAL
  //Assume that the U-Blox GPS is running at 9600 baud (the default) or at 38400 baud.
  //Loop until we're in sync and then ensure it's at 38400 baud.
  do {
    Serial.println("GPS: trying 38400 baud");
    SerialSensor.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);
    if (myGPS.begin(SerialSensor) == true) break;

    delay(100);
    Serial.println("GPS: trying 9600 baud");
    SerialSensor.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    if (myGPS.begin(SerialSensor) == true) {
        Serial.println("GPS: connected at 9600 baud, switching to 38400");
        myGPS.setSerialRate(38400);
        delay(100);
    } else {
        //myGPS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GPS serial connected");

  #else
  I2CSensors.begin(I2C_SDA_2, I2C_SCL_2);
    //Initialize Ticker every 0.5s
  delay(100);
  while (myGPS.begin(I2CSensors, 0x42) == false)
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    //while (1);
  }
  #endif // USE_SERIAL
  #endif // USE_SIMULATOR

  // Keep power when running from battery
  bool isOk = SetPowerBoostKeepOn(1);
  Serial.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  
  
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

  #if USE_SIMULATOR
  Serial.println("Please start outputting NMEA Sentences at serial");
  #else
  #if USE_SERIAL

  myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_NMEA); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1); //Several of these are on by default on virgin ublox board so let's disable them
  myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
  myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
  myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);

  myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
  myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1); //Only leaving GGA/VTG enabled at current navigation rate
  
  #else


  myGPS.setI2COutput(COM_TYPE_NMEA); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C); //Several of these are on by default on virgin ublox board so let's disable them
  myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
  myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
  myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);

  myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
  myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C); //Only leaving GGA/VTG enabled at current navigation rate
  #endif  // USE SERIAL
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  #endif // USE_SIMULATOR
  
  
  
  // Create the BLE Device
  BLEDevice::init("");
  ble_addr = (BLEDevice::getAddress().getNative());

  Serial.printf("String BLE Address: %s", BLEDevice::getAddress().toString().c_str());
  pAdvertising = BLEDevice::getAdvertising();
  //pAdvertising->setDeviceAddress(*ble_addr, BLE_ADDR_TYPE_PUBLIC);
  //SetBeacon();

  basicID.IDType = ODID_IDTYPE_CAA_REGISTRATION_ID;
  basicID.UAType = ODID_UATYPE_ROTORCRAFT;
  char id[] = "ABCDF1FG916003A12345";
  strncpy(basicID.UASID, id, sizeof(id));
  encodeBasicIDMessage(&basicID_enc, &basicID);


  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 300000, true);
  // Start an alarm
  timerAlarmEnable(timer);
  
}

void loop()
{
  #if USE_SIMULATOR
  while(SerialSensor.available() > 0 ){
    char incoming = SerialSensor.read();
    Serial.printf("recieve %c at Serial\n", incoming);
    nmea.process(incoming);
    nmeaBufferforPrint[bufferIdx++] = incoming;
  }
  #else
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.
  #endif
  if(nmea.isValid() == true)
  {
    bufferIdx = 0;
    Serial.println(nmeaBufferforPrint);
    memset(nmeaBufferforPrint, 0, 100);
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();
    long alt = 0;
    nmea.getAltitude(alt);
    long speed = nmea.getSpeed();
    long course = nmea.getCourse();
    gps_data.longitude = longitude_mdeg / 1000000.;
    gps_data.latitude = latitude_mdeg / 1000000.;
    gps_data.altitudeMSL = alt / 1000.;
    //gps_data.altitudeWGS84 = gps_data.altitudeMSL + nmea.getGeodicSeperation()/10.;
    gps_data.altitudeWGS84 = gps_data.altitudeMSL + nmea.getGeodicSeperation()/10.;
    gps_data.course = course / 1000.;
    gps_data.speed = (speed / 1000.) / 1.944;
    GetTimeStampData();

    Serial.println ("GPS Data is: ");
    Serial.print("Latitude (deg): ");
    Serial.print(gps_data.latitude);
    Serial.print("\tLongitude (deg): ");
    Serial.println(gps_data.longitude);
    Serial.print("AltitudeMSL (m): ");
    Serial.print(gps_data.altitudeMSL);
    Serial.print("\tAltitude_WGS84 (m): ");
    Serial.print(gps_data.altitudeWGS84);
    Serial.print("\tspeed (m/s): ");
    Serial.print(gps_data.speed);
    Serial.print("\tCourse (deg): ");
    Serial.println(gps_data.course);
    Serial.println();
    PreparePacketData(&beaconData, &gps_data);
    encodeLocationMessage(&beaconDataEncoded, &beaconData);

    if (now - lastMeasure > 5000) { // update database every 5sec after recieveing the valid NMEA
      lastMeasure = now;
      UpdateDataBase();
    }
  }
  else
  {
    Serial.print("No Fix - ");
    Serial.print("Num. satellites: ");
    Serial.println(nmea.getNumSatellites());
  }

  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    
    bool isLocation = false;
    uint32_t counter = 0;
    uint32_t isrTime= 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isLocation = updateBasicID;
    counter = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    Serial.printf("ISR COunter is %d", counter);
    Serial.print(" itat ");
    Serial.print(isrTime);
    Serial.println(" ms");
    if(isLocation) {
      Serial.println("Updating Static Data");
      setBeaconIDData(&basicID_enc);
      pAdvertising->start();
      Serial.println("Advertizing started...");
      delay(20);
      pAdvertising->stop();
      Serial.println("Advertizing stoped!");

      
    }
    else {
      Serial.println("Updating Dynamic Data");
      setBeaconLocationData(&beaconDataEncoded);
      pAdvertising->start();
      Serial.println("Advertizing started...");
      delay(20);
      pAdvertising->stop();
      Serial.println("Advertizing stoped!");

    }
  }
  delay(30); //Don't pound too hard on the I2C bus
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
  nmeaBufferforPrint[bufferIdx++] = incoming;
}


bool SetPowerBoostKeepOn(int en) 
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


float GetTimeStampData() {
  float timeStamp = 0;

  uint8_t seconds = nmea.getSecond();
  uint8_t minutes = nmea.getMinute();
  uint8_t hours = nmea.getHour();
  uint8_t hundredth = nmea.getHundredths();
  Serial.printf("Time in Nmea sentance is %d:%d:%d.%d\n", hours, minutes, seconds, hundredth);
  timeStamp = (minutes * 600) + (seconds * 10) + (float)(hundredth / 10.);
  Serial.printf("TimeStamp value is:  %f\n\n", timeStamp);
  return timeStamp;
}

void PreparePacketData(ODID_Location_data * inData, GPS_Data_t* p_gps_data) {

  inData->Status = ODID_STATUS_AIRBORNE;
  inData->Direction =(p_gps_data->course > 0)?((p_gps_data->course >= 360)? 361: p_gps_data->course): 0;
  inData->SpeedHorizontal = (p_gps_data->speed > 0)?((p_gps_data->speed >= 254.25)? 254.25: p_gps_data->speed): 0; //If speed is >= 254.25 m/s: 254.25m/s
  inData->SpeedVertical = 0;
  inData->Latitude = p_gps_data->latitude;
  inData->Longitude = p_gps_data->longitude;
  inData->AltitudeBaro = p_gps_data->altitudeMSL;
  inData->AltitudeGeo = p_gps_data->altitudeWGS84;
  inData->HeightType = ODID_HEIGHT_REF_OVER_GROUND;
  inData->Height = 0;
  inData->HorizAccuracy = createEnumHorizontalAccuracy(2.5f);
  inData->VertAccuracy = createEnumVerticalAccuracy(0.5f);
  inData->BaroAccuracy = createEnumVerticalAccuracy(1.5f);
  inData->SpeedAccuracy = createEnumSpeedAccuracy(0.5f);
  inData->TSAccuracy = createEnumTimestampAccuracy(0.2f);
  inData->TimeStamp = GetTimeStampData();
  
}


void setBeaconLocationData(ODID_Location_encoded *PEncodedLocation){

  static uint8_t adCounnter = 0;
  uint8_t adv_data_len = 31;
  char adv_data[adv_data_len];

  adv_data[0] = 0x1E;                     // Preamble
  adv_data[1] = 0xFF;                     // ACC Addr byte 1
  adv_data[2] = 0x02;                     // ACC Addr byte 2
  adv_data[3] = 0x00;                     // ACC Addr byte 3
  adv_data[4] = 0x0D;                     // ACC Addr byte 4
  adv_data[5] = adCounnter++;
  memcpy(&adv_data[6], PEncodedLocation, 25);

  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  //BLEAdvertisementData oScanResponseData = BLEAdvertisementData();
  
  oAdvertisementData.addData(std::string((char*)adv_data, adv_data_len));

  const char * str = std::string((char*)adv_data, adv_data_len).c_str();
  Serial.print("Packet is 0x");
  for (uint8_t i = 0; i < adv_data_len; i++) {
    Serial.printf("%02X", str[i]);
  }
  Serial.println();


  pAdvertising->setAdvertisementData(oAdvertisementData);
  //pAdvertising->setScanResponseData(oScanResponseData);

}


void setBeaconIDData(ODID_BasicID_encoded* pBasicID_enc) {

  static uint8_t adCounnter = 0;
  uint8_t adv_data_len = 31;
  char adv_data[adv_data_len];

  adv_data[0] = 0x1E;                     // Preamble
  adv_data[1] = 0xFF;                     // ACC Addr byte 1
  adv_data[2] = 0x02;                     // ACC Addr byte 2
  adv_data[3] = 0x00;                     // ACC Addr byte 3
  adv_data[4] = 0x0D;                     // ACC Addr byte 4
  adv_data[5] = adCounnter++;
  memcpy(&adv_data[6], pBasicID_enc, 25);

  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
 // BLEAdvertisementData oScanResponseData = BLEAdvertisementData();
  
  oAdvertisementData.addData(std::string((char*)adv_data, adv_data_len));

  const char * str = std::string((char*)adv_data, adv_data_len).c_str();
  Serial.print("Packet is 0x");
  for (uint8_t i = 0; i < adv_data_len; i++) {
    Serial.printf("%02X", str[i]);
  }
  Serial.println();


  pAdvertising->setAdvertisementData(oAdvertisementData);
  //pAdvertising->setScanResponseData(oScanResponseData);


}


void UpdateDataBase()
{
  Serial.print("Connecting to APN: ");
  Serial.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" fail");
  }
  else {
    Serial.println(" OK");
    
    Serial.print("Connecting to ");
    Serial.print(server);
    if (!gsmClient.connect(server, port)) {
      Serial.println(" fail");
    }
    else {
      Serial.println(" OK");
      // Making an HTTP POST request
      Serial.println("Performing HTTP POST request...");
      // Prepare your HTTP POST request data (Temperature in Celsius degrees)
      String httpRequestData = "lat=" + String(gps_data.latitude)
                             + "&lon=" + String(gps_data.longitude) + "&broadcastid=" + String(id) + "";
      
      gsmClient.print(String("POST ") + resource + " HTTP/1.1\r\n");
      gsmClient.print(String("Host: ") + server + "\r\n");
      gsmClient.println("Connection: close");
      gsmClient.println("Content-Type: application/x-www-form-urlencoded");
      gsmClient.print("Content-Length: ");
      gsmClient.println(httpRequestData.length());
      gsmClient.println();
      gsmClient.println(httpRequestData);

      unsigned long timeout = millis();
      while (gsmClient.connected() && millis() - timeout < 10000L) {
        // Print available data (HTTP response from server)
        while (gsmClient.available()) {
          char c = gsmClient.read();
          Serial.print(c);
          timeout = millis();
        }
      }
      Serial.println();
    
      // Close client and disconnect
      gsmClient.stop();
      Serial.println(F("Server disconnected"));
      modem.gprsDisconnect();
      Serial.println(F("GPRS disconnected"));
    }
  }
}