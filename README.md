# thingspeak-esp32-dht11
A ThingSpeak client for the ESP32 + DHT11 sensor

This is a simple, minimal ThingSpeak client. It uses
- THe RMT perhiperal to talk to the DHT11 sensor
- The ESP Wifi support to connect to the Internet
- ESP-IDF Open SSL library to speak HTTP to the ThinkSpeak API
- The ESP's deep sleep to save power between readings.

Configuring
-----------
As with all the ESP-IDF examples, run "make menuconfig".

As well as the normal SDK setup (e.g. Serial port for programming) you 
also need to set the values under ThingSpeak "ESP DHT11 Configuration"

To do this you will need the ThingSpeak API write key for your channel, 
and and the WiFi settings.

Basic program flow
------------------
- Wake up from deep sleep
- Schedule the next wakeup for 5 minutes in the future
- Read sensor
- If it fails, retry reading sensor
- If we have new data, launch the ThingSpeak update task
- Wait for 10 seconds (for wifi task to do its work), and go to deep sleep

The code uses the standard ESP logging infrastructure to log all the process to the console 

Hardware setup
--------------
- Connect the DHT power to Gnd and 3V connections on the ESP32
- Connect the signal line from the DHT11 to the pin 5 - can be changed in app_main()
- If one is not on your DHT11 sensor module, add a sutiable pullup resistor between DATA and 3V connections