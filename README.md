# thingspeak-esp32-dht11
A ThingSpeak client for the ESP32 + DHT11 sensor

This is a simple, minimal ThingSpeak client. It uses
- THe RMT perhiperal to talk to the DHT11 sensor
- The ESP Wifi support to connect to the Internet
- ESP-IDF Open SSL library to speak HTTP to the ThinkSpeak API
- The ESP's deep sleep to save power between readings.

Basic flow
----------
- Wake up from deep sleep
- Schedule the next wakeup for 5 minutes in the future
- Read sensor
- If it fails, retry reading sensor
- If we have new data, launch the ThingSpeak update task
- Wait for 10 seconds (for wifi task to do its work), and go to deep sleep

The details required to configure it to your use-case are:
- The Wifi SSID
- The Wifi Password
- The ThinkSpeak channel's API key
- Which pin the DHT11's sensor is connected to

The code uses the standard ESP logging infrastructure to log all the process to the console 

Hardware setup
--------------
- Connect the DHT power to Gnd and 3V connections on the ESP32
- Connect the signal line from the DHT11 to the pin defined in the config file
- If one is not on your DHT11 module, add a sutiable pullup resistor between DATA and 3V connections
