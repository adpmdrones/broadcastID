# BROADCAST REMOTE DRONE ID
This is the project created for client LucaBrizzi
* You can use app_release_2.apk

## I2C Connections
* The Zoe-M8Q is connected using I2C with the pin connection as shown
* Connect SDA pin of ZOE-M8Q with ESP32 GPIO 18
* Connect SCL pin of ZOE-M8Q with ESP32 GPIO 19


## Serial Connections
* The Zoe-M8Q is connected using I2C with the pin connection as shown
* Connect TX pin of ZOE-M8Q with ESP32 GPIO 14
* Connect RX pin of ZOE-M8Q with ESP32 GPIO 27




## How to use the code
* Install  `Visual Studio Code`
* Install `PlateformIO ` extention for visual studio code.
* After installation open/start  `Visual Studio Code` and `PlateformIO` screen will automatically apears.
* Open the Project.(select Broadcast Drone Folder).
* You can use three modes. 
  * USE SIMULATOR
   ```cpp
    // at line 23 and 24 do this
    #define USE_SIMULATOR     1
    #define USE_SERIAL        1
    // RX and TX pins can be changed at line 81 82
    #define RX_PIN      14 // connect FTDI/ USB TO UART TX pin here
    #define TX_PIN      12 // connect FTDI/ USB TO UART RX pin here
    // USE correct Comport at Simulator, with default settings i.e (Speed = 9600)
   ```
  * USE SERIAL
   ```cpp
    // at line 23 and 24 do this
    #define USE_SIMULATOR     0
    #define USE_SERIAL        1
    // RX and TX pins can be changed at line 81 82
    #define RX_PIN      14 // connect ZOE-M8Q UART TX pin here
    #define TX_PIN      12 // connect  ZOE-M8Q UART RX pin here
   ```
  * USE I2C
  ```cpp
    // at line 23 and 24 do this
    #define USE_SIMULATOR     0
    #define USE_SERIAL        0
    // RX and TX pins can be changed at line 43 44
    #define I2C_SDA_2            18 // connect ZOE-M8Q SDA PIN
    #define I2C_SCL_2            19 // connect ZOE-M8Q SCL PIN
   ```
* Compile it. flash it and run it. 
* You would be able to see Serial Monitor as well.
* ![](/Images/Image1.png)

 
## Deliverable CheckList
|                         Tasks                        | Implementation |               Issues              |
|:----------------------------------------------------:|:--------------:|:---------------------------------:|
|            Interfacing Zoe-M8Q with Esp32            |      DONE      | Tested with I2C. Issues with Uart |
| Implement BLE Beacon according to Opendroneid Format |      DONE      |      Tested with NRF Sniffer      |
|              Implment APP and DO Testing             |      DONE      |         NOT Receving Data         |
|               Interface  NMEA Simulator              |      DONE      |                                   |
|               Interface GSM with Server              |  Partial Done  |  Webserver is not implemented yet |

## Note
I have checked that but in my case Serial is not working. I have check even with a simple example, did factory reset and other thing but with serial my module not working. with I2C it is working fine. So you can check with your Module. Even the module don't work, the static data is sent via BLE after 3seconds and Dynamic data with location related positon as 0 sent out to APP after 300ms (3 messages in a sacond). So if it worked for you then it is really good but you can see the drone posioton in any case because the data is still encoded as recognizable by opendroid app even it is zero.

2nd, about **Nmea Simulator**, although you can use it for testing but it just output NMEA sentences at the begining and then don't set. So stop Simulator, restart the device and then when restarting complete start Nmea simulator again. Although it won't effect the output since once a correct Nmea sentence is recieved the code send that via BLE untill new correct sentence is recieved.