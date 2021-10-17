# Inkplate6-Weather-Display-433MHz

This fork is based on David Bird's excellent e-Paper weather display. https://github.com/G6EJD
![image](https://user-images.githubusercontent.com/13219057/136691834-09dc2aaf-f500-43ec-a5f3-33e2905c53aa.png)

I have added code to receive Temperature & Humidity readings from up to 3x F007TH Thermometer / Hygrometers. https://www.aliexpress.com/item/4000094242136.html?spm=a2g0s.9042311.0.0.58e04c4dYY6Wdu
![image](https://user-images.githubusercontent.com/13219057/136692731-2ec6b61a-d280-44a5-bafb-24b549e8b36c.png)

The 433MHz Receive & Decode is based on the work by:

Rob Ward (whose Manchester Encoding reading by delay rather than interrupt is the basis of this code) https://github.com/robwlakes/ArduinoWeatherOS

The work of 3zero8 capturing and analysing the F007th data http://forum.arduino.cc/index.php?topic=214436.0

The work of Volgy capturing and analysing the F007th data https://github.com/volgy/gr-ambient

Marco Schwartz for showing how to send sensor data to websites http://www.openhomeautomation.net/


Additional Features:

*  433MHz Temperature / Humidity receive. Channels 1, 2 & 3 (Set by DIP switches on Tx) are displayed.
  Strings for Label1, 2, 3 are included in owncredentials2.h.  You can change these to reflect what Channel 1,2,3 represent.
  If there is no 433MHz Thermometer detected on Channel 2, it uses a value from the Inkplate6 built in thermometer
  If there is no 433MHz Thermometer detected on Channel 3, it uses a value received from OpenWeather.com as the outdoor temperature
    
    
*  Web portal for configuration.  Runs when no WiFi credentials are stored or for 1 min on power up.
  You will need to go to https://openweathermap.org/ set up a free account & get an API Key.
  There's also a list of recognised cities here: http://bulk.openweathermap.org/sample/
  If you are outside the UK, you can change the country in owm_credentials2.h
    



![image](https://user-images.githubusercontent.com/13219057/137620945-a7c4e0cb-0801-46e1-8f79-671dfe9aa2ea.png)

In the sketch, I've connected the DATA from the Rx to IO D13. This is the YELLOW wire in the photo below. I suggest you use a Superhetrodyne type receiver such as a RXB6 module - they work much better than the others. Although some references say the RXB6 needs 5v, it works happily on 3.3v too.
I've also used IO D12 to power the Receiver.  This is useful if you want the display to be battery powered - you can de-power the Rx. 
![alt text](https://github.com/SimonRafferty/ESP32-42e-Paper-Weather-Display-/blob/master/Weather%20Rx.jpg)
**<pre>
  int RxPin           = 13;   //The number of signal from the Rx  
  int Enable433       = 12;   //Set to high to power up 433 Board
</pre>**

The WHITE wire is the Antenna. I've used a 1/4 wave, 23cm length of wire for this which achieves a range of over 100m I just formed a loose loop inside the casing.





Obtain your OWM API key - it's free  https://openweathermap.org/

__Note: If you use your API key to receive data too often (once per 30 mins seems OK), it will start blocking you!__

Edit the owm_credentials2.h file in the IDE (TAB at top of IDE) and change your Language, Country, choose your units Metric or Imperial and be sure to find a valid weather station location on OpenWeatherMap, if your display has all blank values your location does not exist!. OPEN WEATHER MAP VERSION - STREAMING JSON

This version requires *** Arduino JSON v6 or above *** it streams the Openweather and processes the data as received, rather than downloading all the data first, then decoding it. It uses a lot less memory this way.

You will need to add the following Libraries to the Arduino IDE

**<pre>
  ArduinoJson      From https://github.com/bblanchon/ArduinoJson  
  Inkplate         This will be installed, when you add the Inkplate 6 Board
</pre>**

The other includes are built-in.

Go to Sketch > Include Library... > Manage Libraries.... Then, for each library, put its name into the text field and the IDE will find it.
