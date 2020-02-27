# BROADCAST REMOTE DRONE ID
This is the project created for client LucaBrizzi
* You can use app_release_2.apk

# I2C Connections
* The Zoe-M8Q is connected using I2C with the pin connection as shown
* Connect SDA pin of ZOE-M8Q with ESP32 GPIO 18
* Connect SCL pin of ZOE-M8Q with ESP32 GPIO 19


# Serial Connections
* The Zoe-M8Q is connected using I2C with the pin connection as shown
* Connect TX pin of ZOE-M8Q with ESP32 GPIO 14
* Connect RX pin of ZOE-M8Q with ESP32 GPIO 27

#Note
I have checked that but in my case Serial is not working. I have check even with a simple example, did factory reset and other thing but with serial my module not working. with I2C it is working fine. So you can check with your Module. Even the module don't work, the static data is sent via BLE after 3seconds and Dynamic data with location related positon as 0 sent out to APP after 300ms (3 messages in a sacond). So if it worked for you then it is really good but you can see the drone posioton in any case because the data is still encoded as recognizable by opendroid app even it is zero.


# How to use the code
* Install  `Visual Studio Code`
* Install `PlateformIO ` extention for visual studio code.
* After installation open/start  `Visual Studio Code` and `PlateformIO` screen will automatically apears.
* Open the Project.(select Broadcast Drone Folder).
* Compile it. flash it and run it. 
* You would be able to see Serial Monitor as well.
* ![](/Images/Image1.png)[