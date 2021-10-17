/* 
  ####################################################################################################################################
  ESP32 Weather Display using an EPD 4.2" Display, obtains data from Open Weather Map, decodes it and then displays it.
  
  Board: Inkplate 6 board
  Repository: https://github.com/SimonRafferty/Inkplate6-Weather-Display-433MHz
  
  This software is a fork of, and much of it is Copyright (c) David Bird 2018. All rights to this software are reserved.
  
  Additionally:
  Rob Ward (whose Manchester Encoding reading by delay rather than interrupt is the basis of this code) https://github.com/robwlakes/ArduinoWeatherOS

  The work of 3zero8 capturing and analysing the F007th data http://forum.arduino.cc/index.php?topic=214436.0

  The work of Volgy capturing and analysing the F007th data https://github.com/volgy/gr-ambient

  Marco Schwartz for showing how to send sensor data to websites http://www.openhomeautomation.net/

  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY
  OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  See more at http://www.dsbird.org.uk
*/
#include <Arduino.h>
#include "owm_credentials2.h"  // See 'owm_credentials' tab and enter your OWM API key and set the Wifi SSID and PASSWORD
#include <ArduinoJson.h>       // https://github.com/bblanchon/ArduinoJson  MUST BE VERSION 5
#include <WiFi.h>              // Built-in
#include "time.h"              // Built-in
#include <SPI.h>               // Built-in 
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <EEPROM.h>  //To store the additional parameter data
#include "ArduinoNvs.h"



WiFiClient wifiClient;

int port = 8080;

// wifimanager can run in a blocking mode or a non blocking mode
// Be sure to know how to process loops with no delay() if using non blocking
bool wm_nonblocking = false; // change to true to use non blocking

WiFiManager wm; // global wm instance

bool SaveParameters = false;

#include <Inkplate.h>               //Include Inkplate library to the sketch
//Fonts
#include "FreeSans9pt7b.h"
#include "FreeSans12pt7b.h"
#include "FreeSans18pt7b.h"
#include "FreeSans24pt7b.h"





#define SCREEN_WIDTH  800.0    // Set for landscape mode, don't remove the decimal place!
#define SCREEN_HEIGHT 600.0
#define BITS_PER_PIXEL 3
#define EPD_BLACK 0
#define EPD_GREY 6
#define EPD_WHITE 7
uint16_t palette[] = { 0, 1 };
#define TEXT_COLOUR 7

Inkplate display(INKPLATE_3BIT);    //Create object on Inkplate library and set library to work in grayscale mode (3-bit)

//################  VERSION  ##########################
String version = "21";       // Version of this program
//################ VARIABLES ###########################

const unsigned long UpdateInterval = (15L * 60L - 13) * 1000000L; // Delay between updates, in microseconds, WU allows 500 requests per-day maximum, set to every 15-mins or more
bool LargeIcon   =  true;
bool SmallIcon   =  false;
#define Large  10
#define Small  4
String time_str, Day_time_str, rxtext; // strings to hold time and received weather data;
int    wifisection, displaysection, MoonDay, MoonMonth, MoonYear;
int    Sunrise, Sunset;

bool Received_Forecast_OK = false;
int IntervalCount = 0;
//################ PROGRAM VARIABLES and OBJECTS ################
//Display Settings - these are entered in the Web Portal and retrieved from NVS
String City           = "";
String Label1         = "";
String Label2         = "";
String Label3         = "";
// Use your own API key by signing up for a free developer account at https://openweathermap.org/
String apikey         = "";                   // See: https://openweathermap.org/



typedef struct { // For current Day and Day 1, 2, 3
  String   Period;
  float    Temperature;
  float    Humidity;
  String   Icon;
  float    High;
  float    Low;
  float    Rainfall;
  float    Pressure;
  int      Cloudcover;
  String   Trend;
  float    Winddir;
  float    Windspeed;
  String   Forecast0;
  String   Forecast1;
  String   Forecast2;
  String   Description;
  String   Time;
} Forecast_record_type;

#define max_readings 24

Forecast_record_type  WxConditions[1];
Forecast_record_type  WxForecast[max_readings];

#define autoscale_on  true
#define autoscale_off false
#define barchart_on   true
#define barchart_off  false

float pressure_readings[max_readings]    = {0};
float temperature_readings[max_readings+1] = {0};
float rain_readings[max_readings]        = {0};

//################ VARIABLES FOR 433MHz DATA CAPTURE ###########################
#define MAX_BYTES 7
#define countof(x) (sizeof(x)/sizeof(x[0]))
// Interface Definitions
int RxPin           = 13;   //The number of signal from the Rx
int Enable433       = 12;   //Set to high to power up 433 Board

// Variables for Manchester Receiver Logic:
word    sDelay     = 242;  //Small Delay about 1/4 of bit duration
word    lDelay     = 484;  //Long Delay about 1/2 of bit duration, 1/4 + 1/2 = 3/4
byte    polarity   = 1;    //0 for lo->hi==1 or 1 for hi->lo==1 for Polarity, sets tempBit at start
byte    tempBit    = 1;    //Reflects the required transition polarity
boolean firstZero  = false;//flags when the first '0' is found.
boolean noErrors   = true; //flags if signal does not follow Manchester conventions
//variables for Header detection
byte    headerBits = 10;   //The number of ones expected to make a valid header
byte    headerHits = 0;    //Counts the number of "1"s to determine a header
//Variables for Byte storage
boolean sync0In=true;      //Expecting sync0 to be inside byte boundaries, set to false for sync0 outside bytes
byte    dataByte   = 0xFF; //Accumulates the bit information
byte    nosBits    = 6;    //Counts to 8 bits within a dataByte
byte    maxBytes   = MAX_BYTES;    //Set the bytes collected after each header. NB if set too high, any end noise will cause an error
byte    nosBytes   = 0;    //Counter stays within 0 -> maxBytes
//Variables for multiple packets
byte    bank       = 0;    //Points to the array of 0 to 3 banks of results from up to 4 last data downloads 
byte    nosRepeats = 3;    //Number of times the header/data is fetched at least once or up to 4 times
//Banks for multiple packets if required (at least one will be needed)
byte  manchester[MAX_BYTES];   //Stores banks of manchester pattern decoded on the fly

//433MHz Derrived readings.  All channels are received. Only 1, 2, 3 are displayed
float Channel1_Temp = -99; float Channel1_Hum = 0; 
float Channel2_Temp = -99; float Channel2_Hum = 0; 
float Channel3_Temp = -99; float Channel3_Hum = 0; 
float Channel4_Temp = -99; float Channel4_Hum = 0; 
float Channel5_Temp = -99; float Channel5_Hum = 0; 
float Channel6_Temp = -99; float Channel6_Hum = 0; 
bool Channel1_New = false;
bool Channel2_New = false;
bool Channel3_New = false;
bool Channel4_New = false;
bool Channel5_New = false;
bool Channel6_New = false;

// Variables to prepare recorded values for Ambient

byte stnId = 0;      //Identifies the channel number
int dataType = 0;    //Identifies the Ambient Thermo-Hygrometer code
int Newtemp = 0;
int Newhum = 0;

String Period0 = "";

long RefreshTimer = millis()-30*60*1000; //Force update on power-up
long DisplayTimer = millis()-30*60*1000;
//WiFiClient client; // wifi client object

//#########################################################################################
void setup() {
  bool reconfigure = false;
  display.begin();        //Init library (you should call this function ONLY ONCE)
  NVS.begin();
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP  
  Serial.begin(115200);
  Serial.setDebugOutput(true);  
  //delay(3000);
  Serial.println("\n Starting");

  
  //***** 433MHz Setup *****
  pinMode(RxPin, INPUT);
  //pinMode(Enable433, OUTPUT);

  //If WiFi cannot connect to stored SSID (If Any), start web portal and display instructions
  //If it can connect, start portal access point for 1 minute so you can change settings.  Only happens at power up.
  StartWiFi();
  

  //This was for battery powered mode
  //Serial.println(F("Starting deep-sleep period..."));
  //delay(1000);
  //begin_sleep();
  
}
//#########################################################################################
void loop() { 
  if ((millis()-RefreshTimer) > 15*60*1000) { //Run every 15 mins  OpenWeather only allows 500 requests per day
    SetupTime();
    if (obtain_wx_data("weather"))  { Received_Forecast_OK = DecodeWeather(rxtext, "weather");  }
    if (obtain_wx_data("forecast")) { Received_Forecast_OK = DecodeWeather(rxtext, "forecast"); }
    RefreshTimer = millis();
  }  
  if ((millis()-DisplayTimer) > 5 *60*1000) { //Run every 5 mins  This will update 433MHz data every 5 minutes. Could be more frequent, but display refresh is distracting
    DisplayTimer = millis(); //Reset timer
    display.clearDisplay(); //Clear any data that may have been in (software) frame buffer. 
                              //(NOTE! This does not clean image on screen, it only clears it in the frame buffer inside ESP32).
  
    if (Received_Forecast_OK) {
      Serial.println(F("Wait up to 3 Mins for 433MHz Tx"));
      unsigned long TOut = millis(); //3 minute total timeout 
  
      while ((millis()-TOut)<90000) { // && (Channel1_Temp == -99) && (Channel2_Temp == -99) && (Channel3_Temp == -99)) {
        Get433Readings();  //Wait for a set of readings (may take 3 mins) and returns results in Channel1_Temp, Channel1_Hum   up to 3 channels worth
        //Serial.print(".");
      }
      Draw433(460, 36);                            //Draw data received from 433MHz Sensors
      
      display.clean();        //Clear everything that has previously been on a screen
  
      Serial.println(F("Forecast OK"));
      eraseManchester();  //clear the array to different nos cause if all zeroes it might think that is a valid 3 packets ie all equal
    
  
      //DrawBattery(SCREEN_WIDTH-30, 10);
      Serial.println(F("Send Data to screen"));
      
      Display_Weather();
  
      Channel1_New = false;
      Channel2_New = false;
      Channel3_New = false;
  
    
      //digitalWrite(Enable433, HIGH);
      
      display.display(); 
    }  
  
    
  }

  
}




void displayCurrentAction(String text) {
  display.fillRect(0, 570, 800, 30, 7);
  display.partialUpdate();
  display.setFont(&FreeSans9pt7b);
  drawString(2,595,text,true);
  //display.display();
  display.partialUpdate();
  
}

void Centre_Text(int x, int y, String text) {
//Calculate the size of a string in the current font and Adjust X position to Centre text
  int16_t  x1, y1;
  uint16_t w, h;
  
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  //Erase the text area
  x1 = x-w/2;
  display.fillRect(x1, y1, w, h, 7);
  drawString(x1,y,text,true);
  //display.setCursor(x1, y);
  //display.print(text);
}

void Right_Text(int x, int y, String text) {
//Calculate the size of a string in the current font and Adjust X position to Centre text
  int16_t  x1, y1;
  uint16_t w, h;
  
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  //Erase the text area
  x1 = x-w;
  display.fillRect(x1, y1, w, h, 7);
  drawString(x1,y,text,true);
  //display.setCursor(x1, y);
  //display.print(text);
}
//#########################################################################################
void eraseManchester()
{
    for( int j=0; j < 4; j++)
    { 
        manchester[j]=j;
    }
}

//#########################################################################################
void Get433Readings(){
    tempBit=polarity; //these begin the same for a packet
    noErrors=true;
    firstZero=false;
    headerHits=0;
    nosBits=6;
    nosBytes=0;
  long n433Timeout = millis();
   
   while (noErrors && (nosBytes<maxBytes) && ((millis()-n433Timeout)<60000) )  
    {
      while((digitalRead(RxPin)!=tempBit) && ((millis()-n433Timeout)<60000))
      {
          //pause here until a transition is found
      }//at Data transition, half way through bit pattern, this should be where RxPin==tempBit
      
      delayMicroseconds(sDelay);//skip ahead to 3/4 of the bit pattern
      // 3/4 the way through, if RxPin has changed it is definitely an error
    
      if (digitalRead(RxPin)!=tempBit)
            {
                noErrors=false;//something has gone wrong, polarity has changed too early, ie always an error
      }//exit and retry
      else
            {
          delayMicroseconds(lDelay);
          //now 1 quarter into the next bit pattern,
          if(digitalRead(RxPin)==tempBit) //if RxPin has not swapped, then bitWaveform is swapping
                {
        //If the header is done, then it means data change is occuring ie 1->0, or 0->1
        //data transition detection must swap, so it loops for the opposite transitions
        tempBit = tempBit^1;
          }//end of detecting no transition at end of bit waveform, ie end of previous bit waveform same as start of next bitwaveform
    
          //Now process the tempBit state and make data definite 0 or 1's, allow possibility of Pos or Neg Polarity 
          byte bitState = tempBit ^ polarity;//if polarity=1, invert the tempBit or if polarity=0, leave it alone.
          if(bitState==1) //1 data could be header or packet
                {
        if(!firstZero)
                    {
            headerHits++;
            if (headerHits==headerBits)
                        {
          Serial.print("H");
                        }
        }
        else
                    {
            DecodeReadings(bitState);//already seen first zero so add bit in
        }
          }//end of dealing with ones
          else
                {  //bitState==0 could first error, first zero or packet
              // if it is header there must be no "zeroes" or errors
              if(headerHits<headerBits)
                    {
                  //Still in header checking phase, more header hits required
                  noErrors=false;//landing here means header is corrupted, so it is probably an error
              }//end of detecting a "zero" inside a header
                    else
                    {
                  //we have our header, chewed up any excess and here is a zero
                  if (!firstZero) //if first zero, it has not been found previously
                        {
                            firstZero=true;
                            DecodeReadings(bitState);//Add first zero to bytes 
                            Serial.print("!");
                        }//end of finding first zero
                        else
                        {
                      DecodeReadings(bitState);
                  }//end of adding a zero bit
                    }//end of dealing with a first zero
                }//end of dealing with zero's (in header, first or later zeroes)
            }//end of first error check
        }//end of while noErrors=true and getting packet of bytes
    } 


//#########################################################################################
int StartWiFi() {
//Use WiFi Manager to connect - with built in AP Portal
  //Get any pre-stored values from EEPROM


  String LocalCity = resdStringFromNVS("City");
  String LocalAPIKey = resdStringFromNVS("APIKey");
  String LocalLabel1 = resdStringFromNVS("Label1");
  String LocalLabel2 = resdStringFromNVS("Label2");
  String LocalLabel3 = resdStringFromNVS("Label3");

  Serial.println("Strings from EEPROM");
  Serial.println(LocalCity);
  Serial.println(LocalAPIKey);
  Serial.println(LocalLabel1);
  Serial.println(LocalLabel2);
  Serial.println(LocalLabel3);
  
  //Store them in WM if not empty
  if(LocalCity.length()==0) LocalCity = "Horsham";
  if(LocalAPIKey.length()==0) LocalAPIKey = "5713e242e734e3b05e6b576d6bf01db9";
  if(LocalLabel1.length()==0) LocalLabel1 = "Workshop";
  if(LocalLabel2.length()==0) LocalLabel2 = "Indoors";
  if(LocalLabel3.length()==0) LocalLabel3 = "Outdoors";

  char cCity[LocalCity.length()+1];   LocalCity.toCharArray(cCity, LocalCity.length()+1);
  char cAPIKey[LocalAPIKey.length()+1];   LocalAPIKey.toCharArray(cAPIKey, LocalAPIKey.length()+1);
  char cLabel1[LocalLabel1.length()+1];   LocalLabel1.toCharArray(cLabel1, LocalLabel1.length()+1);
  char cLabel2[LocalLabel2.length()+1];   LocalLabel2.toCharArray(cLabel2, LocalLabel2.length()+1);
  char cLabel3[LocalLabel3.length()+1];   LocalLabel3.toCharArray(cLabel3, LocalLabel3.length()+1);

  WiFiManagerParameter PortalCity("1", "Weather Report City:", cCity, 40);
  WiFiManagerParameter PortalAPIKey("2", "Open Weather API Key:", cAPIKey, 40);
  WiFiManagerParameter PortalLabel1("3", "Label for 433MHz Ch1:", cLabel1, 40);
  WiFiManagerParameter PortalLabel2("4", "Label for 433MHz Ch2:", cLabel2, 40);
  WiFiManagerParameter PortalLabel3("5", "Label for 433MHz Ch3:", cLabel3, 40);

  wm.addParameter(&PortalCity);
  wm.addParameter(&PortalAPIKey);
  wm.addParameter(&PortalLabel1);
  wm.addParameter(&PortalLabel2);
  wm.addParameter(&PortalLabel3);


  //****************** UNCOMMENT THIS TO RESET WiFi & CONFIG SETTINGS ********************************
  wm.resetSettings(); // wipe settings



  






  if(wm_nonblocking) wm.setConfigPortalBlocking(true);
  
  //*********CALLBACKS**********
  wm.setSaveParamsCallback(saveParamCallback);
  //When webserver starts, show instructions on display
  wm.setWebServerCallback(displayInstructions);
  //Called when Wifi settings are saved
  wm.setSaveConfigCallback(saveConfigCallback);

  // custom menu via array or vector
  std::vector<const char *> menu = {"wifi","sep","param","sep","exit"};
  wm.setMenu(menu);

  // set dark theme
  wm.setClass("invert");



  // wm.setConnectTimeout(20); // how long to try to connect for before continuing
  wm.setConfigPortalTimeout(0); // auto close configportal after n seconds. 0 makes it persist until connected
  //wm.setCaptivePortalEnable(false); // disable captive portal redirection
  // wm.setAPClientCheck(true); // avoid timeout if client connected to softap

  // wifi scan settings
  wm.setRemoveDuplicateAPs(true); // do not remove duplicate ap names (true)
  // wm.setMinimumSignalQuality(20);  // set min RSSI (percentage) to show in scans, null = 8%
  // wm.setShowInfoErase(false);      // do not show erase button on info page
  // wm.setScanDispPerc(true);       // show RSSI as percentage not graph icons
  
  //wm.setBreakAfterConfig(true);   // always exit configportal even if wifi save fails

  bool res;
  res = wm.autoConnect("WeatherStation"); // anonymous ap

  if(SaveParameters) {
    //User has asked to save details

    Serial.println("[CALLBACK] saveParamCallback fired");
    Serial.println(PortalCity.getValue());
    Serial.println(PortalAPIKey.getValue());
    Serial.println(PortalLabel1.getValue());
    Serial.println(PortalLabel2.getValue());
    Serial.println(PortalLabel3.getValue());
    //Allow 60 characters for each string
    writeStringToNVS("City",PortalCity.getValue());
    writeStringToNVS("APIKey",PortalAPIKey.getValue());
    writeStringToNVS("Label1",PortalLabel1.getValue());
    writeStringToNVS("Label2",PortalLabel2.getValue());
    writeStringToNVS("Label3",PortalLabel3.getValue());

  }
  if(!res) {
    Serial.println("Failed to connect or hit timeout");
    display.clearDisplay(); //Clear any data that may have been in (software) frame buffer. 
                              //(NOTE! This does not clean image on screen, it only clears it in the frame buffer inside ESP32).
    display.clean();        //Clear everything that has previously been on a screen
  
    display.setTextWrap(false);           //Disable text wraping
    display.setFont(&FreeSans18pt7b);
    Centre_Text(SCREEN_WIDTH / 2, 40, "Failed to connect to WiFi");
  
    
    display.display();
    
    
    return 0;
  } 
  else {
    //if you get here you have connected to the WiFi    
    Serial.println("WiFi connected at: " + String(WiFi.localIP()));
    SaveParameters = false;
    //Start web portal for 30 sec
    wm.setConfigPortalTimeout(60); // auto close configportal after n seconds. 0 makes it persist until connected
    wm.setConfigPortalBlocking(true);
    wm.setWebPortalClientCheck(true);
    wm.startConfigPortal("WeatherStation");

    
    
    
    Serial.println("STOPPING PORTAL");
    if(SaveParameters) {
      //User has asked to save details
  
      Serial.println("[CALLBACK] saveParamCallback fired");
      Serial.println(PortalCity.getValue());
      Serial.println(PortalAPIKey.getValue());
      Serial.println(PortalLabel1.getValue());
      Serial.println(PortalLabel2.getValue());
      Serial.println(PortalLabel3.getValue());
      //Allow 60 characters for each string
      writeStringToNVS("City",PortalCity.getValue());
      writeStringToNVS("APIKey",PortalAPIKey.getValue());
      writeStringToNVS("Label1",PortalLabel1.getValue());
      writeStringToNVS("Label2",PortalLabel2.getValue());
      writeStringToNVS("Label3",PortalLabel3.getValue());
  
    }

    

    
    //Set globals to values from NVS
    City           = resdStringFromNVS("City");
    Label1         = resdStringFromNVS("Label1");
    Label2         = resdStringFromNVS("Label2");
    Label3         = resdStringFromNVS("Label3");
    apikey         = resdStringFromNVS("APIKey");          // See: https://openweathermap.org    

    Serial.println("NVS Settings Recovered:");
    Serial.print("City: "); Serial.println(City);
    Serial.print("apikey: "); Serial.println(apikey);
    Serial.print("Label 1: "); Serial.println(Label1);
    Serial.print("Label 2: "); Serial.println(Label2);
    Serial.print("Label 3: "); Serial.println(Label3);
    
    return 1;
  }
  
  
}

//#########################################################################################
String getParam(String name){
  //read parameter from server, for customhmtl input
  String value;
  if(wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}
//#########################################################################################

String urlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;   
}

//#########################################################################################

void saveConfigCallback(){
  display.clearDisplay(); //Clear any data that may have been in (software) frame buffer. 
                            //(NOTE! This does not clean image on screen, it only clears it in the frame buffer inside ESP32).
  display.clean();        //Clear everything that has previously been on a screen

  display.setTextWrap(false);           //Disable text wraping
  display.setFont(&FreeSans18pt7b);
  Centre_Text(SCREEN_WIDTH / 2, 40, "WiFi Settings Saved");
  display.setFont(&FreeSans12pt7b);
  Centre_Text(SCREEN_WIDTH / 2, 100, "Return to Menu & hit [Exit]");
  Centre_Text(SCREEN_WIDTH / 2, 140, "Weather data will be displayed shortly");

  
  display.display();
  
}

//#########################################################################################

void saveParamCallback(){
  SaveParameters = true;
  display.clearDisplay(); //Clear any data that may have been in (software) frame buffer. 
                            //(NOTE! This does not clean image on screen, it only clears it in the frame buffer inside ESP32).
  display.clean();        //Clear everything that has previously been on a screen

  display.setTextWrap(false);           //Disable text wraping
  display.setFont(&FreeSans18pt7b);
  Centre_Text(SCREEN_WIDTH / 2, 40, "Config Settings Saved");
  display.setFont(&FreeSans12pt7b);
  Centre_Text(SCREEN_WIDTH / 2, 100, "Return to Menu & hit [Exit]");
  Centre_Text(SCREEN_WIDTH / 2, 140, "Weather data will be displayed shortly");

  display.display();
}

//#########################################################################################

void writeStringToNVS(const String &Tag, const String &strToWrite)
{
  //Tag is just an index to the data, a bit like a collection or fieldname
    bool res = NVS.setString(Tag, strToWrite);

    //Hopefully that has saved.  Read back just to be sure
    Serial.print("String written to NVS = "); Serial.println(NVS.getString(Tag));
}
//#########################################################################################
  
String resdStringFromNVS(const String &Tag)
{
  return NVS.getString(Tag);
}

//#########################################################################################

void displayInstructions() {
//Web server has started, show user instructions on display if WiFi was unable to connect

  if(wm.getLastConxResult() != WL_CONNECTED) {
    display.clearDisplay(); //Clear any data that may have been in (software) frame buffer. 
                              //(NOTE! This does not clean image on screen, it only clears it in the frame buffer inside ESP32).
    display.clean();        //Clear everything that has previously been on a screen
  
    display.setTextWrap(false);           //Disable text wraping
    display.setFont(&FreeSans18pt7b);
    Centre_Text(SCREEN_WIDTH / 2, 40, "Weather Station Settings Portal");
    display.setFont(&FreeSans12pt7b);
    Centre_Text(SCREEN_WIDTH / 2, 90, "Look for a WiFi AP Named WeatherStation");
    Centre_Text(SCREEN_WIDTH / 2, 120, "If the captive portal is not displayed");
    Centre_Text(SCREEN_WIDTH / 2, 150, "Browse to 192.168.1.4");
    Centre_Text(SCREEN_WIDTH / 2, 200, "[Setup] for Weather Settings, inc API Key ## DO THIS FIRST ##");
    Centre_Text(SCREEN_WIDTH / 2, 230, "[Configure WiFi] to Enter Wifi credentials");
    Centre_Text(SCREEN_WIDTH / 2, 290, "You must hit [Save] within [Setup] (to save) then");
    Centre_Text(SCREEN_WIDTH / 2, 320, "return to menu, [Configure WiFi] and hit [Save] to exit");
  
    Centre_Text(SCREEN_WIDTH / 2, 400, "For your API Key, register a free account at:");
    Centre_Text(SCREEN_WIDTH / 2, 430, "https://openweathermap.org/");
  
    Centre_Text(SCREEN_WIDTH / 2, 490, "For a list of recognised Cities:");
    Centre_Text(SCREEN_WIDTH / 2, 520, "http://bulk.openweathermap.org/sample/");
  
    display.setFont(&FreeSans9pt7b);
    Centre_Text(SCREEN_WIDTH / 2, SCREEN_HEIGHT-20, "Config AP will run for 1 min on each power-up");
    
    display.display();
  }
  
}



//#########################################################################################
void StopWiFi() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  wifisection    = millis() - wifisection;
}
//#########################################################################################
void SetupTime() {
  configTime(0, 0, "0.uk.pool.ntp.org", "time.nist.gov");
  setenv("TZ", Timezone, 1);
  delay(500);
  UpdateLocalTime();
}
//#########################################################################################
void UpdateLocalTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println(F("Failed to obtain time"));
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  //Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S");     // Displays: Saturday, June 24 2017 14:05:49
  Serial.println(&timeinfo, "%H:%M:%S");                     // Displays: 14:05:49
  char output[30], day_output[30];
  if (Units == "M") {
    strftime(day_output, 30, "%a  %d-%b-%y", &timeinfo);     // Displays: Sat 24-Jun-17
    strftime(output, 30, "(@ %H:%M:%S )", &timeinfo);        // Creates: '@ 14:05:49'
  }
  else {
    strftime(day_output, 30, "%a  %b-%d-%y", &timeinfo);     // Creates: Sat Jun-24-17
    //strftime(output, 30, "(@ %r )", &timeinfo);              // Creates: '@ 2:05:49pm'
    strftime(output, 30, " %r ", &timeinfo);              // Creates: ' 2:05:49pm'
  }
  Day_time_str = day_output;
  time_str     = output;
}
//#########################################################################################
String ConvertUnixTime(int unix_time) {
  struct tm *now_tm;
  int hour, min, second, day, month, year, wday;
  // timeval tv = {unix_time,0};
  time_t tm = unix_time;
  now_tm = localtime(&tm);
  hour   = now_tm->tm_hour;
  min    = now_tm->tm_min;
  second = now_tm->tm_sec;
  wday   = now_tm->tm_wday;
  day    = now_tm->tm_mday;
  month  = now_tm->tm_mon + 1;
  year   = 1900 + now_tm->tm_year; // To get just YY information
  MoonDay   = day;
  MoonMonth = month;
  MoonYear  = year;
  if (Units == "M") {
    time_str =  (hour < 10 ? "0" + String(hour) : String(hour)) + ":" + (min < 10 ? "0" + String(min) : String(min)) + ":" + "  ";                     // HH:MM   05/07/17
    time_str += (day < 10 ? "0" + String(day) : String(day)) + "/" + (month < 10 ? "0" + String(month) : String(month)) + "/" + (year < 10 ? "0" + String(year) : String(year)); // HH:MM   05/07/17
  }
  else {
    String ampm = "am";
    if (hour > 11) ampm = "pm";
    hour = hour % 12; if (hour == 0) hour = 12;
    time_str =  (hour % 12 < 10 ? "0" + String(hour % 12) : String(hour % 12)) + ":" + (min < 10 ? "0" + String(min) : String(min)) + ampm + " ";      // HH:MMam 07/05/17
    time_str += (month < 10 ? "0" + String(month) : String(month)) + "/" + (day < 10 ? "0" + String(day) : String(day)) + "/" + "/" + (year < 10 ? "0" + String(year) : String(year)); // HH:MMpm 07/05/17
  }
  // Returns either '21:12  ' or ' 09:12pm' depending on Units
  //Serial.println(time_str);
  return time_str;
}
//#########################################################################################


void begin_sleep(){
  esp_sleep_enable_timer_wakeup(UpdateInterval);
  esp_deep_sleep_start(); // Sleep for e.g. 30 minutes
}
//#########################################################################################
void Display_Weather() {              // 7" Inkplate6 display is 800x600 resolution
  Draw_Heading_Section();             // Top line of the display
  Draw_Main_Weather_Section(170, 100); // Centre section of display for Location, temperature, Weather report, current Wx Symbol and wind direction
  GetReadingsForGraphs();
  Draw_Forecast_Section(300, 50);     // 3hr forecast boxes
  DrawGraphs(460, 36);
  //Draw_Astronomy_Section(460, 40);    // Astronomy section Sun rise/set, Moon phase and Moon icon
}
//#########################################################################################
void Draw_Heading_Section() {
  display.setTextWrap(false);           //Disable text wraping
  display.setFont(&FreeSans12pt7b);
  Centre_Text(SCREEN_WIDTH / 2, 20, City);

  display.setFont(&FreeSans9pt7b);
  Right_Text(SCREEN_WIDTH-10, 15, Day_time_str);
  
  drawString(5, 15, time_str,true);
  display.drawLine(0, 25, SCREEN_WIDTH, 25,EPD_BLACK);
}
//#########################################################################################

void Draw_Main_Weather_Section(int x, int y) {
  DisplayWXicon(x+40, y-25, WxConditions[0].Icon, LargeIcon);
  display.setFont(&FreeSans12pt7b);
  DrawPressureTrend(x+40, y + 50, WxConditions[0].Pressure, WxConditions[0].Trend);
  //Draw_Rain(x - 100, y + 25);
  display.setFont(&FreeSans12pt7b);
  String Wx_Description = WxConditions[0].Forecast0;
  if (WxConditions[0].Forecast1 != "") Wx_Description += " & " +  WxConditions[0].Forecast1;
    if (WxConditions[0].Forecast2 != "" && WxConditions[0].Forecast1 != WxConditions[0].Forecast2) Wx_Description += " & " +  WxConditions[0].Forecast2;
  Centre_Text(x+40, y + 80, Wx_Description);
  
  Draw_Main_Wx(x -85, y +5);
  display.drawLine(0, y + 90, SCREEN_WIDTH, y + 90,EPD_BLACK);
}
//#########################################################################################
String TitleCase(String text) {
  if (text.length() > 0) {
    String temp_text = text.substring(0, 1);
    temp_text.toUpperCase();
    return temp_text + text.substring(1); // Title-case the string
  }
  return "";
}


void GetReadingsForGraphs() {
  for (int r = 1; r <= max_readings; r++) {
    if (Units == "I") pressure_readings[r] = WxForecast[r].Pressure * 0.02953;  
    else              pressure_readings[r] = WxForecast[r].Pressure;
      temperature_readings[r] = WxForecast[r].Temperature;
    if (Units == "I") rain_readings[r]     = WxForecast[r].Rainfall * 0.0393701;
    else              rain_readings[r]     = WxForecast[r].Rainfall;
  }

}
//#########################################################################################
void Draw_Forecast_Section(int x, int y) {
//3 Day Forecast

  //display.setFont(ArialMT_Plain_10);
  Draw_Forecast_Weather(x, y, 0);
  Draw_Forecast_Weather(x + 83, y, 1);
  Draw_Forecast_Weather(x + 166, y, 2);
  Draw_Forecast_Weather(x + 249, y, 3);
  Draw_Forecast_Weather(x + 332, y, 4);
  Draw_Forecast_Weather(x + 415, y, 5);
  
  //display.drawLine(0, y + 173, SCREEN_WIDTH, y + 173,EPD_BLACK);
  //display.drawLine(0, y + 150, SCREEN_WIDTH, y + 150,EPD_BLACK);
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
  //display.setFont(ArialRoundedMTBold_14);
  //drawString(x - 40, y + 173, "3-Day Forecast Values",true);
}
void Draw433(int x, int y) {
  //#############################
  //Draw 433MHz Local Temp & Humidity
  //display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(&FreeSans9pt7b);
  if((Channel3_Temp == -99)){
    //Substitute OWM Temp for local outdoor temp
    Channel3_Temp = WxConditions[0].Temperature;
    Channel3_Hum = WxConditions[0].Humidity;
    Channel3_New = true;
    drawString(SCREEN_WIDTH/400*278, y + 180, "Outdoor (WWW)",true);
  } else {
    drawString(SCREEN_WIDTH/400*278, y + 180, Label3 + " (Ch 3)",true);
  }
  if((Channel2_Temp == -99)){
    //Substitute on board Temp for local indoor temp
    Channel2_Temp = display.readTemperature();
    Serial.print("Board Temperature "); Serial.println(Channel2_Temp);
    Channel2_Hum = 0;
    Channel2_New = true;
    drawString(SCREEN_WIDTH/400*148, y + 180, "Ambient (Panel)",true);
  } else {
    drawString(SCREEN_WIDTH/400*148, y + 180, Label2 + " (Ch 2)",true);
  }
  drawString(SCREEN_WIDTH/400*10, y + 180, Label1 + " (Ch 1)",true);
  
  
/*
  //Dummy data for test
  
  Channel1_Temp = 20.3;
  Channel1_Hum = 56;
  Channel2_Temp = 20.3;
  Channel2_Hum = 56;
  Channel3_Temp = 20.3;
  Channel3_Hum = 56;
 */
  if(Channel1_Temp != -99){
    display.setFont(&FreeSans24pt7b);
    drawString(SCREEN_WIDTH/400*10, y +240, String(Channel1_Temp,1) + "°",Channel1_New); // Show current Local Temperature
    display.setFont(&FreeSans12pt7b);
    drawString(SCREEN_WIDTH/400*70, y +240, String(Channel1_Hum,0) + "%",Channel1_New); // Show current Local Humidity
  }
  if(Channel2_Temp != -99){
    display.setFont(&FreeSans24pt7b);
    drawString(SCREEN_WIDTH/400*148, y +240, String(Channel2_Temp,1) + "°",Channel2_New); // Show current Local Temperature
    display.setFont(&FreeSans12pt7b);
    drawString(SCREEN_WIDTH/400*208, y +240, String(Channel2_Hum,0) + "%",Channel2_New); // Show current Local Humidity
  }
  if(Channel3_Temp != -99) {
    display.setFont(&FreeSans24pt7b);
    drawString(SCREEN_WIDTH/400*278, y +240, String(Channel3_Temp,1) + "°",Channel3_New); // Show current Local Temperature
    display.setFont(&FreeSans12pt7b);
    drawString(SCREEN_WIDTH/400*338, y +240, String(Channel3_Hum,0) + "%",Channel3_New); // Show current Local Humidity
  }
}
  
void DrawGraphs(int x, int y) {  
  display.drawLine(0, y + 280, SCREEN_WIDTH, y + 280,EPD_BLACK);
  //display.setTextAlignment(TEXT_ALIGN_LEFT);
  //display.setFont(ArialMT_Plain_10);
  for (int r = 1; r <= max_readings; r++) {
    if (Units == "I") pressure_readings[r] = WxForecast[r].Pressure * 0.02953;  
    else              pressure_readings[r] = WxForecast[r].Pressure;
    temperature_readings[r+1] = WxForecast[r].Temperature;
    if (Units == "I") rain_readings[r]     = WxForecast[r].Rainfall * 0.0393701;
    else              rain_readings[r]     = WxForecast[r].Rainfall;
  }
  temperature_readings[1] = WxConditions[0].Temperature; //Replace first prediction with current temperature
  DrawGraph(SCREEN_WIDTH/400*30,  SCREEN_HEIGHT/300*180, SCREEN_WIDTH/400*100, SCREEN_HEIGHT/300*80,900,1050,"Pressure", pressure_readings, max_readings, autoscale_on, barchart_off);
  DrawGraph(SCREEN_WIDTH/400*158, SCREEN_HEIGHT/300*180, SCREEN_WIDTH/400*100, SCREEN_HEIGHT/300*80,10,30, "Temperature", temperature_readings, max_readings, autoscale_on, barchart_off);
  //DrawGraph(SCREEN_WIDTH/400*288, SCREEN_HEIGHT/300*222, SCREEN_WIDTH/400*100, SCREEN_HEIGHT/300*60,0,30, "Rainfall", rain_readings, max_readings, autoscale_on, barchart_on);
  DrawGraph(SCREEN_WIDTH/400*288, SCREEN_HEIGHT/300*180, SCREEN_WIDTH/400*100, SCREEN_HEIGHT/300*80,0,30, "Rainfall", rain_readings, max_readings, autoscale_on, barchart_on);
}
//#########################################################################################
void Draw_Forecast_Weather(int x, int y, int index) {
  display.setFont(&FreeSans9pt7b);
  display.drawRect(x, y, 80, 95,EPD_BLACK);
  display.drawLine(x + 1, y + 25, x + 80, y + 25,EPD_BLACK);
  DisplayWXicon(x + 32, y + 50, WxForecast[index].Icon, SmallIcon);
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(&FreeSans9pt7b);

  if(index!=0) {
    drawString(x + 15, y+20, String(WxForecast[index].Period.substring(11,16)),true);
  } else {
    drawString(x + 15, y+20, String(Period0.substring(11,16)),true);
  }
  drawString(x + 10, y + 90, String(WxForecast[index].High,0) + "° / " + String(WxForecast[index].Low,0) + "°",true);

  
}
//#########################################################################################
void Draw_Main_Wx(int x, int y) {
  DrawWind(x, y, WxConditions[0].Winddir, WxConditions[0].Windspeed);
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
  //display.setFont(ArialRoundedMTBold_14);
  //drawString(x, y - 28, String(WxConditions[0].High,0) + "° | " + String(WxConditions[0].Low,0) + "°"); // Show forecast high and Low
  //display.setFont(ArialMT_Plain_24);
  //drawString(x - 5, y - 10, String(WxConditions[0].Temperature,1) + "°"); // Show current Temperature
  //display.setFont(ArialRoundedMTBold_14);
  ////display.setTextAlignment(TEXT_ALIGN_LEFT);  
  //drawString(x+String(WxConditions[0].Temperature,1).length()*11/2,y-9,Units=="M"?"C":"F"); // Add in smaller Temperature unit
  display.setFont(&FreeSans9pt7b);
  //display.setTextAlignment(TEXT_ALIGN_LEFT);
}
//#########################################################################################
void DrawWind(int x, int y, float angle, float windspeed) {
  int Cradius = 55;
  float dx = Cradius * cos((angle - 90) * PI / 180) + x; // calculate X position
  float dy = Cradius * sin((angle - 90) * PI / 180) + y; // calculate Y position
  arrow(x, y, Cradius - 3, angle, 15, 15); // Show wind direction on outer circle
  display.drawCircle(x, y, Cradius + 2,EPD_BLACK);
  display.drawCircle(x, y, Cradius + 3,EPD_BLACK);
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(&FreeSans12pt7b);
  //drawString(x-Cradius/3, y + Cradius - 15, WindDegToDirection(angle));
  Centre_Text(x, y + Cradius - 15, WindDegToDirection(angle)); 

  
  display.setFont(&FreeSans12pt7b);
  Centre_Text(x, y-14, String(windspeed * 2.244,1)); //Display in mph regardless of units
  //drawString(x-Cradius/3-10, y-14, String(windspeed * 2.244,1)); //Display in mph regardless of units
  display.setFont(&FreeSans9pt7b);
  //drawString(x-Cradius/3-10, y, "mph");
  Centre_Text(x, y, "mph"); 

}
//#########################################################################################
String WindDegToDirection(float winddirection) {
  if (winddirection >= 348.75 || winddirection < 11.25)  return "N";
  if (winddirection >=  11.25 && winddirection < 33.75)  return "NNE";
  if (winddirection >=  33.75 && winddirection < 56.25)  return "NE";
  if (winddirection >=  56.25 && winddirection < 78.75)  return "ENE";
  if (winddirection >=  78.75 && winddirection < 101.25) return "E";
  if (winddirection >= 101.25 && winddirection < 123.75) return "ESE";
  if (winddirection >= 123.75 && winddirection < 146.25) return "SE";
  if (winddirection >= 146.25 && winddirection < 168.75) return "SSE";
  if (winddirection >= 168.75 && winddirection < 191.25) return "S";
  if (winddirection >= 191.25 && winddirection < 213.75) return "SSW";
  if (winddirection >= 213.75 && winddirection < 236.25) return "SW";
  if (winddirection >= 236.25 && winddirection < 258.75) return "WSW";
  if (winddirection >= 258.75 && winddirection < 281.25) return "W";
  if (winddirection >= 281.25 && winddirection < 303.75) return "WNW";
  if (winddirection >= 303.75 && winddirection < 326.25) return "NW";
  if (winddirection >= 326.25 && winddirection < 348.75) return "NNW";
  return "?";
}
//#########################################################################################
void DrawPressureTrend(int x, int y, float pressure, String slope) {
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(&FreeSans18pt7b);
  //drawString(x-50, y-10, String(pressure,0) + (Units == "M" ? "mb" : "in"),true);
  Centre_Text(x, y-10, String(pressure,0) + (Units == "M" ? "mb" : "in")); 
  /*
  x = x + 45; y = y + 8;
  if      (slope == "+") {
    display.drawLine(x,  y,  x + 4, y - 4,EPD_BLACK);
    display.drawLine(x + 4, y - 4, x + 8, y,EPD_BLACK);
    display.drawLine(x,  y+1,  x + 5, y - 3,EPD_BLACK);
    display.drawLine(x + 4, y - 3, x + 8, y+1,EPD_BLACK);
  }
  else if (slope == "0") {
    display.drawLine(x + 3, y - 4, x + 8, y,EPD_BLACK);
    display.drawLine(x + 3, y + 4, x + 8, y,EPD_BLACK);
    display.drawLine(x + 3, y - 3, x + 8, y+1,EPD_BLACK);
    display.drawLine(x + 3, y + 5, x + 8, y+1,EPD_BLACK);
  }
  else if (slope == "-") {
    display.drawLine(x,  y,  x + 4, y + 4,EPD_BLACK);
    display.drawLine(x + 4, y + 4, x + 8, y,EPD_BLACK);
    display.drawLine(x,  y+1,  x + 4, y + 5,EPD_BLACK);
    display.drawLine(x + 4, y + 5, x + 8, y+1,EPD_BLACK);
  }
  */
}
//#########################################################################################
void Draw_Rain(int x, int y) {
  display.setFont(&FreeSans12pt7b);
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
  if (WxForecast[1].Rainfall > 0) drawString(x, y + 14, String(WxForecast[1].Rainfall,2) + (Units == "M" ? "mm" : "in") + " Rainfall",true); // Only display rainfall total today if > 0
  display.setFont(&FreeSans9pt7b);
}
//#########################################################################################
void Draw_Astronomy_Section(int x, int y) {
  //display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(&FreeSans9pt7b);
  display.drawRect(x, y + 64, 167, 53,EPD_BLACK);
  drawString(x + 4, y + 65, "Sun Rise/Set",true);
  drawString(x + 20, y + 75, ConvertUnixTime(Sunrise).substring(0, 5),true);
  drawString(x + 20, y + 86, ConvertUnixTime(Sunset).substring(0, 5),true);
  drawString(x + 4, y + 100, "Moon:",true);
  drawString(x + 35, y + 100, MoonPhase(MoonDay, MoonMonth, MoonYear, Hemisphere),true);
  DrawMoon(x + 103, y + 51, MoonDay, MoonMonth, MoonYear, Hemisphere);
}
//#########################################################################################
void DrawMoon(int x, int y, int dd, int mm, int yy, String hemisphere) {
  int diameter = 38;
  float Xpos, Ypos, Rpos, Xpos1, Xpos2, ip, ag;
  for (Ypos = 0; Ypos <= 45; Ypos++) {
    Xpos = sqrt(45 * 45 - Ypos * Ypos);
    // Draw dark part of moon
    double pB1x = (90   - Xpos) / 90 * diameter + x;
    double pB1y = (Ypos + 90) / 90   * diameter + y;
    double pB2x = (Xpos + 90) / 90   * diameter + x;
    double pB2y = (Ypos + 90) / 90   * diameter + y;
    double pB3x = (90   - Xpos) / 90 * diameter + x;
    double pB3y = (90   - Ypos) / 90 * diameter + y;
    double pB4x = (Xpos + 90) / 90   * diameter + x;
    double pB4y = (90   - Ypos) / 90 * diameter + y;
    display.drawLine(pB1x, pB1y, pB2x, pB2y,EPD_BLACK);
    display.drawLine(pB3x, pB3y, pB4x, pB4y,EPD_BLACK);
    // Determine the edges of the lighted part of the moon
    int j = JulianDate(dd, mm, yy);
    //Calculate the approximate phase of the moon
    double Phase = (j + 4.867) / 29.53059;
    Phase = Phase - (int)Phase;
    if (Phase < 0.5) ag = Phase * 29.53059 + 29.53059 / 2; else ag = Phase * 29.53059 - 29.53059 / 2; // Moon's age in days
    if (hemisphere == "south") Phase = 1 - Phase;
    Rpos = 2 * Xpos;
    if (Phase < 0.5) {
      Xpos1 = - Xpos;
      Xpos2 = (Rpos - 2 * Phase * Rpos - Xpos);
    }
    else {
      Xpos1 = Xpos;
      Xpos2 = (Xpos - 2 * Phase * Rpos + Rpos);
    }
    // Draw light part of moon
    double pW1x = (Xpos1 + 90) / 90 * diameter + x;
    double pW1y = (90 - Ypos) / 90  * diameter + y;
    double pW2x = (Xpos2 + 90) / 90 * diameter + x;
    double pW2y = (90 - Ypos) / 90  * diameter + y;
    double pW3x = (Xpos1 + 90) / 90 * diameter + x;
    double pW3y = (Ypos + 90) / 90  * diameter + y;
    double pW4x = (Xpos2 + 90) / 90 * diameter + x;
    double pW4y = (Ypos + 90) / 90  * diameter + y;
    display.drawLine(pW1x, pW1y, pW2x, pW2y,EPD_WHITE);
    display.drawLine(pW3x, pW3y, pW4x, pW4y,EPD_WHITE);
  }
  display.drawCircle(x + diameter - 1, y + diameter, diameter / 2 + 1,EPD_BLACK);
}
//#########################################################################################
int JulianDate(int d, int m, int y) {
  int mm, yy, k1, k2, k3, j;
  yy = y - (int)((12 - m) / 10);
  mm = m + 9;
  if (mm >= 12) mm = mm - 12;
  k1 = (int)(365.25 * (yy + 4712));
  k2 = (int)(30.6001 * mm + 0.5);
  k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;
  // 'j' for dates in Julian calendar:
  j = k1 + k2 + d + 59 + 1;
  if (j > 2299160) j = j - k3; // 'j' is the Julian date at 12h UT (Universal Time) For Gregorian calendar:
  return j;
}
//#########################################################################################
String MoonPhase(int d, int m, int y, String hemisphere) {
  int c, e;
  double jd;
  int b;
  if (m < 3) {
    y--;
    m += 12;
  }
  ++m;
  c   = 365.25 * y;
  e   = 30.6 * m;
  jd  = c + e + d - 694039.09;           /* jd is total days elapsed */
  jd /= 29.53059;                        /* divide by the moon cycle (29.53 days) */
  b   = jd;                              /* int(jd) -> b, take integer part of jd */
  jd -= b;                               /* subtract integer part to leave fractional part of original jd */
  b   = jd * 8 + 0.5;                    /* scale fraction from 0-8 and round by adding 0.5 */
  b   = b & 7;                           /* 0 and 8 are the same phase so modulo 8 for 0 */
  if (hemisphere == "south") b = 7 - b;
  if (b == 0) return "New";              // New; 0% illuminated
  if (b == 1) return "Waxing crescent";  // Waxing crescent; 25% illuminated
  if (b == 2) return "First quarter";    // First quarter; 50% illuminated
  if (b == 3) return "Waxing gibbous";   // Waxing gibbous; 75% illuminated
  if (b == 4) return "Full";             // Full; 100% illuminated
  if (b == 5) return "Waning gibbous";   // Waning gibbous; 75% illuminated
  if (b == 6) return "Third quarter";    // Last quarter; 50% illuminated
  if (b == 7) return "Waning crescent";  // Waning crescent; 25% illuminated
  return "";
}
//#########################################################################################
void DrawCircle(int x, int y, int Cstart, int Cend, int Cradius, int Coffset_radius, int Coffset) {
  float dx, dy;
  for (int i = Cstart; i < Cend; i++) {
    dx = (Cradius + Coffset_radius) * cos((i - 90) * PI / 180) + x + Coffset / 2; // calculate X position
    dy = (Cradius + Coffset_radius) * sin((i - 90) * PI / 180) + y; // calculate Y position
    display.drawPixel(dx, dy,EPD_BLACK);
  }
}
//#########################################################################################
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength) {
  float dx = (asize - 10) * cos((aangle - 90) * PI / 180) + x; // calculate X position
  float dy = (asize - 10) * sin((aangle - 90) * PI / 180) + y; // calculate Y position
  float x1 = 0;         float y1 = plength;
  float x2 = pwidth / 2;  float y2 = pwidth / 2;
  float x3 = -pwidth / 2; float y3 = pwidth / 2;
  float angle = aangle * PI / 180 - 135;
  float xx1 = x1 * cos(angle) - y1 * sin(angle) + dx;
  float yy1 = y1 * cos(angle) + x1 * sin(angle) + dy;
  float xx2 = x2 * cos(angle) - y2 * sin(angle) + dx;
  float yy2 = y2 * cos(angle) + x2 * sin(angle) + dy;
  float xx3 = x3 * cos(angle) - y3 * sin(angle) + dx;
  float yy3 = y3 * cos(angle) + x3 * sin(angle) + dy;
  display.fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2,EPD_BLACK);
}
//#########################################################################################
void DisplayWXicon(int x, int y, String IconName, bool LargeSize) {
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
  Serial.println(IconName);
    if      (IconName == "01d" || IconName == "01n")  Sunny(x, y, LargeSize, IconName);
    else if (IconName == "02d" || IconName == "02n")  MostlySunny(x, y, LargeSize, IconName);
    else if (IconName == "03d" || IconName == "03n")  Cloudy(x, y, LargeSize, IconName);
    else if (IconName == "04d" || IconName == "04n")  MostlySunny(x, y, LargeSize, IconName);
    else if (IconName == "09d" || IconName == "09n")  ChanceRain(x, y, LargeSize, IconName);
    else if (IconName == "10d" || IconName == "10n")  Rain(x, y, LargeSize, IconName);
    else if (IconName == "11d" || IconName == "11n")  Tstorms(x, y, LargeSize, IconName); 
    else if (IconName == "13d" || IconName == "13n")  Snow(x, y, LargeSize, IconName);
    else if (IconName == "50d")                       Haze(x, y - 5, LargeSize, IconName);
    else if (IconName == "50n")                       Fog(x, y - 5, LargeSize, IconName);
    else                                              Nodata(x, y, LargeSize);
}

//#########################################################################################
bool obtain_wx_data(String RequestType) {
  rxtext = "";
  String units = (Units == "M"?"metric":"imperial");
  //wifiClient.stop(); // close connection before sending a new request
  Serial.print("connecting to: "); Serial.println(server);
  if (wifiClient.connect(server, 80)) { // if the connection succeeds
      Serial.print("CONNECTED to: "); Serial.println(server);
    // send the HTTP PUT request:
      Serial.println("GET /data/2.5/" + RequestType + "?q=" + City + "," + Country + "&APPID=" + apikey + "&mode=json&units="+units+"&lang="+Language+" HTTP/1.1");
    if (RequestType == "weather")
      wifiClient.println("GET /data/2.5/" + RequestType + "?q=" + urlencode(City) + "," + urlencode(Country) + "&APPID=" + apikey + "&mode=json&units="+units+"&lang="+Language+" HTTP/1.1");
    else
      wifiClient.println("GET /data/2.5/" + RequestType + "?q=" + urlencode(City) + "," + urlencode(Country) + "&APPID=" + apikey + "&mode=json&units="+units+"&lang="+Language+"&cnt=24 HTTP/1.1");
    wifiClient.println("Host: api.openweathermap.org");
    wifiClient.println("User-Agent: ESP OWM Receiver/1.1");
    wifiClient.println("Connection: close");
    wifiClient.println();
    unsigned long timeout = millis();
    while (wifiClient.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println(">>> Client Timeout !");
        //wifiClient.stop();
        return false;
      }
    }
    char c = 0;
    bool startJson = false;
    int jsonend = 0;
    while (wifiClient.available()) {
      c = wifiClient.read();
      Serial.print(c);
      // JSON formats contain an equal number of open and close curly brackets, so check that JSON is received correctly by counting open and close brackets
      if (c == '{') {
        startJson = true; // set true to indicate JSON message has started
        jsonend++;
      }
      if (c == '}') {
        jsonend--;
      }
      if (startJson == true) {
        rxtext += c; // Add in the received character
      }
      // if jsonend = 0 then we have have received equal number of curly braces
      if (jsonend == 0 && startJson == true) {
        Serial.println("Received OK...");
        //Serial.println(rxtext);
        //wifiClient.stop();
        return true;
      }
    }
  }
  else {
    // if no connction was made:
    Serial.println("connection failed");
    return false;
  }
  rxtext = "";
  return true;
}
//#########################################################################################
// Problems with stucturing JSON decodes, see here: https://arduinojson.org/assistant/
bool DecodeWeather(String json, String Type) {
  Serial.print(F("Creating JSON object...and "));
  DynamicJsonBuffer jsonBuffer (30*1024);
  JsonObject& root = jsonBuffer.parseObject(const_cast<char*>(json.c_str()));
  if (!root.success()) {
    Serial.print("Parse JSON Object() failed");
    display.clearDisplay(); //Clear any data that may have been in (software) frame buffer. 
                              //(NOTE! This does not clean image on screen, it only clears it in the frame buffer inside ESP32).
    display.clean();        //Clear everything that has previously been on a screen
  
    display.setTextWrap(false);           //Disable text wraping
    display.setFont(&FreeSans18pt7b);
    Centre_Text(SCREEN_WIDTH / 2, 40, "Sorry, but either your API Key");
    Centre_Text(SCREEN_WIDTH / 2, 80, "or Home City were not recognised");
    Centre_Text(SCREEN_WIDTH / 2, 120, "Reset your settings & try again!");
  
    
    display.display();
    
    return false;
  }
  Serial.println(" Decoding JSON " + Type + " data");
  if (Type == "weather") {
    //Serial.println(json);
    // All Serial.println statements are for diagnostic purposes and not required, remove if not needed 
    String lon                  = root["coord"]["lon"];                                                                 Serial.println("Lon:" + lon);
    String lat                  = root["coord"]["lat"];                                                                 Serial.println("Lat:" + lat);
    JsonObject& weather         = root["weather"][0];
    const char* main0           = weather["main"];       if (main0 != NULL) {WxConditions[0].Forecast0 = String(main0); Serial.println(WxConditions[0].Forecast0);}
    const char* icon            = weather["icon"];       if (icon  != NULL) {WxConditions[0].Icon      = String(icon);  Serial.println(WxConditions[0].Icon);}
    JsonObject& weather1        = root["weather"][1];
    const char* main1           = weather1["main"];      if (main1 != NULL) {WxConditions[0].Forecast1 = String(main1); Serial.println(WxConditions[0].Forecast1);}
    JsonObject& weather2        = root["weather"][2];            
    const char* main2           = weather1["main"];      if (main2 != NULL) {WxConditions[0].Forecast2 = String(main2); Serial.println(WxConditions[0].Forecast2);}
    JsonObject& main            = root["main"];
    WxConditions[0].Temperature = main["temp"];          Serial.println(WxConditions[0].Temperature);
    WxConditions[0].Pressure    = main["pressure"];      Serial.println(WxConditions[0].Pressure);
    WxConditions[0].Humidity    = main["humidity"];      Serial.println(WxConditions[0].Humidity);
    WxConditions[0].Low         = main["temp_min"];      Serial.println(WxConditions[0].Low);
    WxConditions[0].High        = main["temp_max"];      Serial.println(WxConditions[0].High);
    WxConditions[0].Windspeed   = root["wind"]["speed"]; Serial.println(WxConditions[0].Windspeed);
    WxConditions[0].Winddir     = root["wind"]["deg"];   Serial.println(WxConditions[0].Winddir);
    WxConditions[0].Cloudcover  = root["clouds"]["all"]; Serial.println(WxConditions[0].Cloudcover); // in % of cloud cover
    JsonObject& sys             = root["sys"];
    String country              = sys["country"];        Serial.println(country);
    int    sunrise              = sys["sunrise"];        Serial.println(sunrise);
    int    sunset               = sys["sunset"];         Serial.println(sunset);
    Sunrise = sunrise;
    Sunset  = sunset;
  }
  if (Type == "forecast") {
    //Serial.println(json);
    Serial.print("\nReceiving Forecast period - 0,"); //------------------------------------------------
    const char* cod                 = root["cod"]; // "200"
    float message                   = root["message"]; 
    int cnt                         = root["cnt"]; 
    JsonArray& list                 = root["list"];

    JsonObject& list0               = list[0];
    int list0_dt                    = list0["dt"]; 
    JsonObject& list0_main          = list0["main"];
    WxForecast[0].Temperature       = list0_main["temp"];
    WxForecast[0].Low               = list0_main["temp_min"];
    WxForecast[0].High              = list0_main["temp_max"];
    WxForecast[0].Pressure          = list0_main["pressure"];
    WxForecast[0].Humidity          = list0_main["humidity"];
    JsonObject& list0_weather0      = list0["weather"][0];
    const char * list0_forecast     = list0_weather0["main"];        if (list0_forecast != NULL) WxForecast[0].Forecast0 = String(list0_forecast);
    const char * list0_description  = list0_weather0["description"]; if (list0_description != NULL) WxForecast[0].Description = String(list0_description);
    const char * list0_icon         = list0_weather0["icon"];        if (list0_icon != NULL) WxForecast[0].Icon = String(list0_icon);
    WxForecast[0].Windspeed         = list0["wind"]["speed"];
    WxForecast[0].Winddir           = list0["wind"]["deg"];
    WxForecast[0].Rainfall          = list0["rain"]["3h"];
    const char * list0_period       = list0["dt_txt"];               if (list0_period != NULL) WxForecast[0].Period = String(list0_period);
    Period0 = WxForecast[0].Period;
    
    Serial.print("1,"); //------------------------------------------------
    JsonObject& list1               = list[1];
    long list1_dt                   = list1["dt"]; 
    JsonObject& list1_main          = list1["main"];
    WxForecast[1].Temperature       = list1_main["temp"];
    WxForecast[1].Low               = list1_main["temp_min"];
    WxForecast[1].High              = list1_main["temp_max"];
    WxForecast[1].Pressure          = list1_main["pressure"];
    WxForecast[1].Humidity          = list1_main["humidity"];
    JsonObject& list1_weather0      = list1["weather"][0];
    const char * list1_forecast     = list1_weather0["main"];        if (list1_forecast != NULL) WxForecast[1].Forecast0 = String(list1_forecast);
    const char * list1_description  = list1_weather0["description"]; if (list1_description != NULL) WxForecast[1].Description = String(list1_description);
    const char * list1_icon         = list1_weather0["icon"];        if (list1_icon != NULL) WxForecast[1].Icon = String(list1_icon);
    WxForecast[1].Windspeed         = list1["wind"]["speed"];
    WxForecast[1].Winddir           = list1["wind"]["deg"];
    WxForecast[1].Rainfall          = list1["rain"]["3h"];
    const char * list1_period       = list1["dt_txt"];               if (list1_period != NULL) WxForecast[1].Period = String(list1_period);

    Serial.print("2,"); //------------------------------------------------
    JsonObject& list2               = list[2];
    long list2_dt                   = list2["dt"]; 
    JsonObject& list2_main          = list2["main"];
    WxForecast[2].Temperature       = list2_main["temp"];
    WxForecast[2].Low               = list2_main["temp_min"];
    WxForecast[2].High              = list2_main["temp_max"];
    WxForecast[2].Pressure          = list2_main["pressure"];
    WxForecast[2].Humidity          = list2_main["humidity"]; 
    JsonObject& list2_weather0      = list2["weather"][0];
    const char * list2_forecast     = list2_weather0["main"];        if (list2_forecast != NULL) WxForecast[2].Forecast0 = String(list2_forecast);
    const char * list2_description  = list2_weather0["description"]; if (list2_description != NULL) WxForecast[2].Description = String(list2_description);
    const char * list2_icon         = list2_weather0["icon"];        if (list2_icon != NULL) WxForecast[2].Icon = String(list2_icon);
    WxForecast[2].Windspeed         = list2["wind"]["speed"]; 
    WxForecast[2].Winddir           = list2["wind"]["deg"]; 
    WxForecast[2].Rainfall          = list2["rain"]["3h"]; 
    const char * list2_period       = list2["dt_txt"];               if (list2_period != NULL) WxForecast[2].Period = String(list2_period);

    Serial.print("3,"); //------------------------------------------------
    JsonObject& list3               = list[3];
    long list3_dt                   = list3["dt"]; 
    JsonObject& list3_main          = list3["main"];
    WxForecast[3].Temperature       = list3_main["temp"];
    WxForecast[3].Low               = list3_main["temp_min"];
    WxForecast[3].High              = list3_main["temp_max"];
    WxForecast[3].Pressure          = list3_main["pressure"];
    WxForecast[3].Humidity          = list3_main["humidity"]; 
    JsonObject& list3_weather0      = list3["weather"][0];
    const char * list3_forecast     = list3_weather0["main"];        if (list3_forecast != NULL) WxForecast[3].Forecast0 = String(list3_forecast);
    const char * list3_description  = list3_weather0["description"]; if (list3_description != NULL)WxForecast[3].Description = String(list3_description);
    const char * list3_icon         = list3_weather0["icon"];        if (list3_icon != NULL)WxForecast[3].Icon = String(list3_icon);
    WxForecast[3].Windspeed         = list3["wind"]["speed"]; 
    WxForecast[3].Winddir           = list3["wind"]["deg"]; 
    WxForecast[3].Rainfall          = list3["rain"]["3h"]; 
    const char * list3_period       = list3["dt_txt"];               if (list3_period != NULL)WxForecast[3].Period = String(list3_period);

    Serial.print("4,");  //---------------------------
    JsonObject& list4               = list[4];
    long list4_dt                   = list4["dt"];
    JsonObject& list4_main          = list4["main"];
    WxForecast[4].Temperature       = list4_main["temp"];
    WxForecast[4].Low               = list4_main["temp_min"];
    WxForecast[4].High              = list4_main["temp_max"];
    WxForecast[4].Pressure          = list4_main["pressure"];
    WxForecast[4].Humidity          = list4_main["humidity"];
    JsonObject& list4_weather0      = list4["weather"][0];
    const char * list4_forecast     = list4_weather0["main"];        if (list4_forecast != NULL) WxForecast[4].Forecast0 = String(list4_forecast);
    const char * list4_description  = list4_weather0["description"]; if (list4_description != NULL) WxForecast[4].Description = String(list4_description);
    const char * list4_icon         = list4_weather0["icon"];        if (list4_icon != NULL) WxForecast[4].Icon = String(list4_icon);
    WxForecast[4].Windspeed         = list4["wind"]["speed"];
    WxForecast[4].Winddir           = list4["wind"]["deg"];
    WxForecast[4].Rainfall          = list4["rain"]["3h"];
    const char * list4_period       = list4["dt_txt"];               if (list4_period != NULL) WxForecast[4].Period = String(list4_period);

    Serial.print("5,");  //---------------------------
    JsonObject& list5               = list[5];
    long list5_dt                   = list5["dt"];
    JsonObject& list5_main          = list5["main"];
    WxForecast[5].Temperature       = list5_main["temp"];
    WxForecast[5].Low               = list5_main["temp_min"];
    WxForecast[5].High              = list5_main["temp_max"];
    WxForecast[5].Pressure          = list5_main["pressure"];
    WxForecast[5].Humidity          = list5_main["humidity"];
    JsonObject& list5_weather0      = list5["weather"][0];
    const char * list5_forecast     = list5_weather0["main"];        if (list5_forecast != NULL) WxForecast[5].Forecast0 = String(list5_forecast);
    const char * list5_description  = list5_weather0["description"]; if (list5_description != NULL) WxForecast[5].Description = String(list5_description);
    const char * list5_icon         = list5_weather0["icon"];        if (list5_icon != NULL) WxForecast[5].Icon = String(list5_icon);
    WxForecast[5].Windspeed         = list5["wind"]["speed"];
    WxForecast[5].Winddir           = list5["wind"]["deg"];
    WxForecast[5].Rainfall          = list5["rain"]["3h"];
    const char * list5_period       = list5["dt_txt"];               if (list5_period != NULL) WxForecast[5].Period = String(list5_period);

    Serial.print("6,");  //---------------------------
    JsonObject& list6               = list[6];
    long list6_dt                   = list6["dt"];
    JsonObject& list6_main          = list6["main"];
    WxForecast[6].Temperature       = list6_main["temp"];
    WxForecast[6].Low               = list6_main["temp_min"];
    WxForecast[6].High              = list6_main["temp_max"];
    WxForecast[6].Pressure          = list6_main["pressure"];
    WxForecast[6].Humidity          = list6_main["humidity"];
    JsonObject& list6_weather0      = list6["weather"][0];
    const char * list6_forecast     = list6_weather0["main"];        if (list6_forecast != NULL) WxForecast[6].Forecast0 = String(list6_forecast);
    const char * list6_description  = list6_weather0["description"]; if (list6_description != NULL) WxForecast[6].Description = String(list6_description);
    const char * list6_icon         = list6_weather0["icon"];        if (list6_icon != NULL) WxForecast[6].Icon = String(list6_icon);
    WxForecast[6].Windspeed         = list6["wind"]["speed"];
    WxForecast[6].Winddir           = list6["wind"]["deg"];
    WxForecast[6].Rainfall          = list6["rain"]["3h"];
    const char * list6_period       = list6["dt_txt"];               if (list6_period != NULL) WxForecast[6].Period = String(list6_period);

    Serial.print("7,");  //---------------------------
    JsonObject& list7               = list[7];
    long list7_dt                   = list7["dt"];
    JsonObject& list7_main          = list7["main"];
    WxForecast[7].Temperature       = list7_main["temp"];
    WxForecast[7].Low               = list7_main["temp_min"];
    WxForecast[7].High              = list7_main["temp_max"];
    WxForecast[7].Pressure          = list7_main["pressure"];
    WxForecast[7].Humidity          = list7_main["humidity"];
    JsonObject& list7_weather0      = list7["weather"][0];
    const char * list7_forecast     = list7_weather0["main"];        if (list7_forecast != NULL) WxForecast[7].Forecast0 = String(list7_forecast);
    const char * list7_description  = list7_weather0["description"]; if (list7_description != NULL) WxForecast[7].Description = String(list7_description);
    const char * list7_icon         = list7_weather0["icon"];        if (list7_icon != NULL) WxForecast[7].Icon = String(list7_icon);
    WxForecast[7].Windspeed         = list7["wind"]["speed"];
    WxForecast[7].Winddir           = list7["wind"]["deg"];
    WxForecast[7].Rainfall          = list7["rain"]["3h"];
    const char * list7_period       = list7["dt_txt"];               if (list7_period != NULL) WxForecast[7].Period = String(list7_period);

    Serial.print("8,");  //---------------------------
    JsonObject& list8               = list[8];
    long list8_dt                   = list8["dt"];
    JsonObject& list8_main          = list8["main"];
    WxForecast[8].Temperature       = list8_main["temp"];
    WxForecast[8].Low               = list8_main["temp_min"];
    WxForecast[8].High              = list8_main["temp_max"];
    WxForecast[8].Pressure          = list8_main["pressure"];
    WxForecast[8].Humidity          = list8_main["humidity"];
    JsonObject& list8_weather0      = list8["weather"][0];
    const char * list8_forecast     = list8_weather0["main"];        if (list8_forecast != NULL) WxForecast[8].Forecast0 = String(list8_forecast);
    const char * list8_description  = list8_weather0["description"]; if (list8_description != NULL) WxForecast[8].Description = String(list8_description);
    const char * list8_icon         = list8_weather0["icon"];        if (list8_icon != NULL) WxForecast[8].Icon = String(list8_icon);
    WxForecast[8].Windspeed         = list8["wind"]["speed"];
    WxForecast[8].Winddir           = list8["wind"]["deg"];
    WxForecast[8].Rainfall          = list8["rain"]["3h"];
    const char * list8_period       = list8["dt_txt"];               if (list8_period != NULL) WxForecast[8].Period = String(list8_period);

    Serial.print("9,");  //---------------------------
    JsonObject& list9               = list[9];
    long list9_dt                   = list9["dt"];
    JsonObject& list9_main          = list9["main"];
    WxForecast[9].Temperature       = list9_main["temp"];
    WxForecast[9].Low               = list9_main["temp_min"];
    WxForecast[9].High              = list9_main["temp_max"];
    WxForecast[9].Pressure          = list9_main["pressure"];
    WxForecast[9].Humidity          = list9_main["humidity"];
    JsonObject& list9_weather0      = list9["weather"][0];
    const char * list9_forecast     = list9_weather0["main"];        if (list9_forecast != NULL) WxForecast[9].Forecast0 = String(list9_forecast);
    const char * list9_description  = list9_weather0["description"]; if (list9_description != NULL) WxForecast[9].Description = String(list9_description);
    const char * list9_icon         = list9_weather0["icon"];        if (list9_icon != NULL) WxForecast[9].Icon = String(list9_icon);
    WxForecast[9].Windspeed         = list9["wind"]["speed"];
    WxForecast[9].Winddir           = list9["wind"]["deg"];
    WxForecast[9].Rainfall          = list9["rain"]["3h"];
    const char * list9_period       = list9["dt_txt"];               if (list9_period != NULL) WxForecast[9].Period = String(list9_period);

    Serial.print("10,");  //---------------------------
    JsonObject& list10               = list[10];
    long list10_dt                   = list10["dt"];
    JsonObject& list10_main          = list10["main"];
    WxForecast[10].Temperature       = list10_main["temp"];
    WxForecast[10].Low               = list10_main["temp_min"];
    WxForecast[10].High              = list10_main["temp_max"];
    WxForecast[10].Pressure          = list10_main["pressure"];
    WxForecast[10].Humidity          = list10_main["humidity"];
    JsonObject& list10_weather0      = list10["weather"][0];
    const char * list10_forecast     = list10_weather0["main"];        if (list10_forecast != NULL) WxForecast[10].Forecast0 = String(list10_forecast);
    const char * list10_description  = list10_weather0["description"]; if (list10_description != NULL) WxForecast[10].Description = String(list10_description);
    const char * list10_icon         = list10_weather0["icon"];        if (list10_icon != NULL) WxForecast[10].Icon = String(list10_icon);
    WxForecast[10].Windspeed         = list10["wind"]["speed"];
    WxForecast[10].Winddir           = list10["wind"]["deg"];
    WxForecast[10].Rainfall          = list10["rain"]["3h"];
    const char * list10_period       = list10["dt_txt"];               if (list10_period != NULL) WxForecast[10].Period = String(list10_period);

    Serial.print("11,");  //---------------------------
    JsonObject& list11               = list[11];
    long list11_dt                   = list11["dt"];
    JsonObject& list11_main          = list11["main"];
    WxForecast[11].Temperature       = list11_main["temp"];
    WxForecast[11].Low               = list11_main["temp_min"];
    WxForecast[11].High              = list11_main["temp_max"];
    WxForecast[11].Pressure          = list11_main["pressure"];
    WxForecast[11].Humidity          = list11_main["humidity"];
    JsonObject& list11_weather0      = list11["weather"][0];
    const char * list11_forecast     = list11_weather0["main"];        if (list11_forecast != NULL) WxForecast[11].Forecast0 = String(list11_forecast);
    const char * list11_description  = list11_weather0["description"]; if (list11_description != NULL) WxForecast[11].Description = String(list11_description);
    const char * list11_icon         = list11_weather0["icon"];        if (list11_icon != NULL) WxForecast[11].Icon = String(list11_icon);
    WxForecast[11].Windspeed         = list11["wind"]["speed"];
    WxForecast[11].Winddir           = list11["wind"]["deg"];
    WxForecast[11].Rainfall          = list11["rain"]["3h"];
    const char * list11_period       = list11["dt_txt"];               if (list11_period != NULL) WxForecast[11].Period = String(list11_period);

    Serial.print("12,");  //---------------------------
    JsonObject& list12               = list[12];
    long list12_dt                   = list12["dt"];
    JsonObject& list12_main          = list12["main"];
    WxForecast[12].Temperature       = list12_main["temp"];
    WxForecast[12].Low               = list12_main["temp_min"];
    WxForecast[12].High              = list12_main["temp_max"];
    WxForecast[12].Pressure          = list12_main["pressure"];
    WxForecast[12].Humidity          = list12_main["humidity"];
    JsonObject& list12_weather0      = list12["weather"][0];
    const char * list12_forecast     = list12_weather0["main"];        if (list12_forecast != NULL) WxForecast[12].Forecast0 = String(list12_forecast);
    const char * list12_description  = list12_weather0["description"]; if (list12_description != NULL) WxForecast[12].Description = String(list12_description);
    const char * list12_icon         = list12_weather0["icon"];        if (list12_icon != NULL) WxForecast[12].Icon = String(list12_icon);
    WxForecast[12].Windspeed         = list12["wind"]["speed"];
    WxForecast[12].Winddir           = list12["wind"]["deg"];
    WxForecast[12].Rainfall          = list12["rain"]["3h"];
    const char * list12_period       = list12["dt_txt"];               if (list12_period != NULL) WxForecast[12].Period = String(list12_period);

    Serial.print("13,");  //---------------------------
    JsonObject& list13               = list[13];
    long list13_dt                   = list13["dt"];
    JsonObject& list13_main          = list13["main"];
    WxForecast[13].Temperature       = list13_main["temp"];
    WxForecast[13].Low               = list13_main["temp_min"];
    WxForecast[13].High              = list13_main["temp_max"];
    WxForecast[13].Pressure          = list13_main["pressure"];
    WxForecast[13].Humidity          = list13_main["humidity"];
    JsonObject& list13_weather0      = list13["weather"][0];
    const char * list13_forecast     = list13_weather0["main"];        if (list13_forecast != NULL) WxForecast[13].Forecast0 = String(list13_forecast);
    const char * list13_description  = list13_weather0["description"]; if (list13_description != NULL) WxForecast[13].Description = String(list13_description);
    const char * list13_icon         = list13_weather0["icon"];        if (list13_icon != NULL) WxForecast[13].Icon = String(list13_icon);
    WxForecast[13].Windspeed         = list13["wind"]["speed"];
    WxForecast[13].Winddir           = list13["wind"]["deg"];
    WxForecast[13].Rainfall          = list13["rain"]["3h"];
    const char * list13_period       = list13["dt_txt"];               if (list13_period != NULL) WxForecast[13].Period = String(list13_period);

    Serial.print("14,");  //---------------------------
    JsonObject& list14               = list[14];
    long list14_dt                   = list14["dt"];
    JsonObject& list14_main          = list14["main"];
    WxForecast[14].Temperature       = list14_main["temp"];
    WxForecast[14].Low               = list14_main["temp_min"];
    WxForecast[14].High              = list14_main["temp_max"];
    WxForecast[14].Pressure          = list14_main["pressure"];
    WxForecast[14].Humidity          = list14_main["humidity"];
    JsonObject& list14_weather0      = list14["weather"][0];
    const char * list14_forecast     = list14_weather0["main"];        if (list14_forecast != NULL) WxForecast[14].Forecast0 = String(list14_forecast);
    const char * list14_description  = list14_weather0["description"]; if (list14_description != NULL) WxForecast[14].Description = String(list14_description);
    const char * list14_icon         = list14_weather0["icon"];        if (list14_icon != NULL) WxForecast[14].Icon = String(list14_icon);
    WxForecast[14].Windspeed         = list14["wind"]["speed"];
    WxForecast[14].Winddir           = list14["wind"]["deg"];
    WxForecast[14].Rainfall          = list14["rain"]["3h"];
    const char * list14_period       = list14["dt_txt"];               if (list14_period != NULL) WxForecast[14].Period = String(list14_period);

    Serial.print("15,");  //---------------------------
    JsonObject& list15               = list[15];
    long list15_dt                   = list15["dt"];
    JsonObject& list15_main          = list15["main"];
    WxForecast[15].Temperature       = list15_main["temp"];
    WxForecast[15].Low               = list15_main["temp_min"];
    WxForecast[15].High              = list15_main["temp_max"];
    WxForecast[15].Pressure          = list15_main["pressure"];
    WxForecast[15].Humidity          = list15_main["humidity"];
    JsonObject& list15_weather0      = list15["weather"][0];
    const char * list15_forecast     = list15_weather0["main"];        if (list15_forecast != NULL) WxForecast[15].Forecast0 = String(list15_forecast);
    const char * list15_description  = list15_weather0["description"]; if (list15_description != NULL) WxForecast[15].Description = String(list15_description);
    const char * list15_icon         = list15_weather0["icon"];        if (list15_icon != NULL) WxForecast[15].Icon = String(list15_icon);
    WxForecast[15].Windspeed         = list15["wind"]["speed"];
    WxForecast[15].Winddir           = list15["wind"]["deg"];
    WxForecast[15].Rainfall          = list15["rain"]["3h"];
    const char * list15_period       = list15["dt_txt"];               if (list15_period != NULL) WxForecast[15].Period = String(list15_period);

    Serial.print("16,");  //---------------------------
    JsonObject& list16               = list[16];
    long list16_dt                   = list16["dt"];
    JsonObject& list16_main          = list16["main"];
    WxForecast[16].Temperature       = list16_main["temp"];
    WxForecast[16].Low               = list16_main["temp_min"];
    WxForecast[16].High              = list16_main["temp_max"];
    WxForecast[16].Pressure          = list16_main["pressure"];
    WxForecast[16].Humidity          = list16_main["humidity"];
    JsonObject& list16_weather0      = list16["weather"][0];
    const char * list16_forecast     = list16_weather0["main"];        if (list16_forecast != NULL) WxForecast[16].Forecast0 = String(list16_forecast);
    const char * list16_description  = list16_weather0["description"]; if (list16_description != NULL) WxForecast[16].Description = String(list16_description);
    const char * list16_icon         = list16_weather0["icon"];        if (list16_icon != NULL) WxForecast[16].Icon = String(list16_icon);
    WxForecast[16].Windspeed         = list16["wind"]["speed"];
    WxForecast[16].Winddir           = list16["wind"]["deg"];
    WxForecast[16].Rainfall          = list16["rain"]["3h"];
    const char * list16_period       = list16["dt_txt"];               if (list16_period != NULL) WxForecast[16].Period = String(list16_period);

    Serial.print("17,");  //---------------------------
    JsonObject& list17               = list[17];
    long list17_dt                   = list17["dt"];
    JsonObject& list17_main          = list17["main"];
    WxForecast[17].Temperature       = list17_main["temp"];
    WxForecast[17].Low               = list17_main["temp_min"];
    WxForecast[17].High              = list17_main["temp_max"];
    WxForecast[17].Pressure          = list17_main["pressure"];
    WxForecast[17].Humidity          = list17_main["humidity"];
    JsonObject& list17_weather0      = list17["weather"][0];
    const char * list17_forecast     = list17_weather0["main"];        if (list17_forecast != NULL) WxForecast[17].Forecast0 = String(list17_forecast);
    const char * list17_description  = list17_weather0["description"]; if (list17_description != NULL) WxForecast[17].Description = String(list17_description);
    const char * list17_icon         = list17_weather0["icon"];        if (list17_icon != NULL) WxForecast[17].Icon = String(list17_icon);
    WxForecast[17].Windspeed         = list17["wind"]["speed"];
    WxForecast[17].Winddir           = list17["wind"]["deg"];
    WxForecast[17].Rainfall          = list17["rain"]["3h"];
    const char * list17_period       = list17["dt_txt"];               if (list17_period != NULL) WxForecast[17].Period = String(list17_period);

    Serial.print("18,");  //---------------------------
    JsonObject& list18               = list[18];
    long list18_dt                   = list18["dt"];
    JsonObject& list18_main          = list18["main"];
    WxForecast[18].Temperature       = list18_main["temp"];
    WxForecast[18].Low               = list18_main["temp_min"];
    WxForecast[18].High              = list18_main["temp_max"];
    WxForecast[18].Pressure          = list18_main["pressure"];
    WxForecast[18].Humidity          = list18_main["humidity"];
    JsonObject& list18_weather0      = list18["weather"][0];
    const char * list18_forecast     = list18_weather0["main"];        if (list18_forecast != NULL) WxForecast[18].Forecast0 = String(list18_forecast);
    const char * list18_description  = list18_weather0["description"]; if (list18_description != NULL) WxForecast[18].Description = String(list18_description);
    const char * list18_icon         = list18_weather0["icon"];        if (list18_icon != NULL) WxForecast[18].Icon = String(list18_icon);
    WxForecast[18].Windspeed         = list18["wind"]["speed"];
    WxForecast[18].Winddir           = list18["wind"]["deg"];
    WxForecast[18].Rainfall          = list18["rain"]["3h"];
    const char * list18_period       = list18["dt_txt"];               if (list18_period != NULL) WxForecast[18].Period = String(list18_period);

    Serial.print("19,");  //---------------------------
    JsonObject& list19               = list[19];
    long list19_dt                   = list19["dt"];
    JsonObject& list19_main          = list19["main"];
    WxForecast[19].Temperature       = list19_main["temp"];
    WxForecast[19].Low               = list19_main["temp_min"];
    WxForecast[19].High              = list19_main["temp_max"];
    WxForecast[19].Pressure          = list19_main["pressure"];
    WxForecast[19].Humidity          = list19_main["humidity"];
    JsonObject& list19_weather0      = list19["weather"][0];
    const char * list19_forecast     = list19_weather0["main"];        if (list19_forecast != NULL) WxForecast[19].Forecast0 = String(list19_forecast);
    const char * list19_description  = list19_weather0["description"]; if (list19_description != NULL) WxForecast[19].Description = String(list19_description);
    const char * list19_icon         = list19_weather0["icon"];        if (list19_icon != NULL) WxForecast[19].Icon = String(list19_icon);
    WxForecast[19].Windspeed         = list19["wind"]["speed"];
    WxForecast[19].Winddir           = list19["wind"]["deg"];
    WxForecast[19].Rainfall          = list19["rain"]["3h"];
    const char * list19_period       = list19["dt_txt"];               if (list19_period != NULL) WxForecast[19].Period = String(list19_period);

    Serial.print("20,");  //---------------------------
    JsonObject& list20               = list[20];
    long list20_dt                   = list20["dt"];
    JsonObject& list20_main          = list20["main"];
    WxForecast[20].Temperature       = list20_main["temp"];
    WxForecast[20].Low               = list20_main["temp_min"];
    WxForecast[20].High              = list20_main["temp_max"];
    WxForecast[20].Pressure          = list20_main["pressure"];
    WxForecast[20].Humidity          = list20_main["humidity"];
    JsonObject& list20_weather0      = list20["weather"][0];
    const char * list20_forecast     = list20_weather0["main"];        if (list20_forecast != NULL) WxForecast[20].Forecast0 = String(list20_forecast);
    const char * list20_description  = list20_weather0["description"]; if (list20_description != NULL) WxForecast[20].Description = String(list20_description);
    const char * list20_icon         = list20_weather0["icon"];        if (list20_icon != NULL) WxForecast[20].Icon = String(list20_icon);
    WxForecast[20].Windspeed         = list20["wind"]["speed"];
    WxForecast[20].Winddir           = list20["wind"]["deg"];
    WxForecast[20].Rainfall          = list20["rain"]["3h"];
    const char * list20_period       = list20["dt_txt"];               if (list20_period != NULL) WxForecast[20].Period = String(list20_period);

    Serial.print("21,");  //---------------------------
    JsonObject& list21               = list[21];
    long list21_dt                   = list21["dt"];
    JsonObject& list21_main          = list21["main"];
    WxForecast[21].Temperature       = list21_main["temp"];
    WxForecast[21].Low               = list21_main["temp_min"];
    WxForecast[21].High              = list21_main["temp_max"];
    WxForecast[21].Pressure          = list21_main["pressure"];
    WxForecast[21].Humidity          = list21_main["humidity"];
    JsonObject& list21_weather0      = list21["weather"][0];
    const char * list21_forecast     = list21_weather0["main"];        if (list21_forecast != NULL) WxForecast[21].Forecast0 = String(list21_forecast);
    const char * list21_description  = list21_weather0["description"]; if (list21_description != NULL) WxForecast[21].Description = String(list21_description);
    const char * list21_icon         = list21_weather0["icon"];        if (list21_icon != NULL) WxForecast[21].Icon = String(list21_icon);
    WxForecast[21].Windspeed         = list21["wind"]["speed"];
    WxForecast[21].Winddir           = list21["wind"]["deg"];
    WxForecast[21].Rainfall          = list21["rain"]["3h"];
    const char * list21_period       = list21["dt_txt"];               if (list21_period != NULL) WxForecast[21].Period = String(list21_period);

    Serial.print("22,");  //---------------------------
    JsonObject& list22               = list[22];
    long list22_dt                   = list22["dt"];
    JsonObject& list22_main          = list22["main"];
    WxForecast[22].Temperature       = list22_main["temp"];
    WxForecast[22].Low               = list22_main["temp_min"];
    WxForecast[22].High              = list22_main["temp_max"];
    WxForecast[22].Pressure          = list22_main["pressure"];
    WxForecast[22].Humidity          = list22_main["humidity"];
    JsonObject& list22_weather0      = list22["weather"][0];
    const char * list22_forecast     = list22_weather0["main"];        if (list22_forecast != NULL) WxForecast[22].Forecast0 = String(list22_forecast);
    const char * list22_description  = list22_weather0["description"]; if (list22_description != NULL) WxForecast[22].Description = String(list22_description);
    const char * list22_icon         = list22_weather0["icon"];        if (list22_icon != NULL) WxForecast[22].Icon = String(list22_icon);
    WxForecast[22].Windspeed         = list22["wind"]["speed"];
    WxForecast[22].Winddir           = list22["wind"]["deg"];
    WxForecast[22].Rainfall          = list22["rain"]["3h"];
    const char * list22_period       = list22["dt_txt"];               if (list22_period != NULL) WxForecast[22].Period = String(list22_period);

    Serial.print("23 ");  //---------------------------
    JsonObject& list23               = list[23];
    long list23_dt                   = list23["dt"];
    JsonObject& list23_main          = list23["main"];
    WxForecast[23].Temperature       = list23_main["temp"];
    WxForecast[23].Low               = list23_main["temp_min"];
    WxForecast[23].High              = list23_main["temp_max"];
    WxForecast[23].Pressure          = list23_main["pressure"];
    WxForecast[23].Humidity          = list23_main["humidity"];
    JsonObject& list23_weather0      = list23["weather"][0];
    const char * list23_forecast     = list23_weather0["main"];        if (list23_forecast != NULL) WxForecast[23].Forecast0 = String(list23_forecast);
    const char * list23_description  = list23_weather0["description"]; if (list23_description != NULL) WxForecast[23].Description = String(list23_description);
    const char * list23_icon         = list23_weather0["icon"];        if (list23_icon != NULL) WxForecast[23].Icon = String(list23_icon);
    WxForecast[23].Windspeed         = list23["wind"]["speed"];
    WxForecast[23].Winddir           = list23["wind"]["deg"];
    WxForecast[23].Rainfall          = list23["rain"]["3h"];
    const char * list23_period       = list23["dt_txt"];               if (list23_period != NULL) WxForecast[23].Period = String(list23_period);

    Serial.println(esp_get_free_heap_size());

    for (int i = 0; i < 24; i++){
      Serial.println("\nPeriod-"+String(i));
      Serial.println(WxForecast[i].Temperature);
      Serial.println(WxForecast[i].Low);
      Serial.println(WxForecast[i].High);
      Serial.println(WxForecast[i].Pressure);
      Serial.println(WxForecast[i].Humidity);
      Serial.println(WxForecast[i].Forecast0);
      Serial.println(WxForecast[i].Forecast1);
      Serial.println(WxForecast[i].Forecast2);
      Serial.println(WxForecast[i].Icon);
      Serial.println(WxForecast[i].Windspeed);
      Serial.println(WxForecast[i].Winddir);
      Serial.println(WxForecast[i].Rainfall);
      Serial.println(WxForecast[i].Period);
    }    

    //------------------------------------------
    float pressure_trend = WxForecast[1].Pressure - WxForecast[5].Pressure; // Measure pressure slope between ~now and later
    pressure_trend = ((int)(pressure_trend * 10)) / 10.0; // Remove any small variations less than 0.1
    WxConditions[0].Trend = "0";
    if (pressure_trend > 0)  WxConditions[0].Trend = "+";
    if (pressure_trend < 0)  WxConditions[0].Trend = "-";
    if (pressure_trend == 0) WxConditions[0].Trend = "0";

    if (Units == "I") Convert_Readings_to_Imperial();
  }
  return true;
}
//#########################################################################################
void Convert_Readings_to_Imperial() {
  WxConditions[0].Pressure    = WxConditions[0].Pressure * 0.02953; //  hPa to ins
  WxForecast[1].Rainfall      = WxForecast[1].Rainfall * 0.0393701; // mm to inches of rainfall
}


//#########################################################################################
// Symbols are drawn on a relative 10x10grid and 1 scale unit = 1 drawing unit
void addcloud(int x, int y, int scale, int linesize) {
  //Draw cloud outer
  display.fillCircle(x - scale * 3, y, scale,EPD_BLACK);                       // Left most circle
  display.fillCircle(x + scale * 3, y, scale,EPD_BLACK);                       // Right most circle
  display.fillCircle(x - scale, y - scale, scale * 1.4,EPD_BLACK);             // left middle upper circle
  display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75,EPD_BLACK); // Right middle upper circle
  display.fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1,EPD_BLACK); // Upper and lower lines
  //Clear cloud inner
  display.fillCircle(x - scale * 3, y, scale - linesize,EPD_GREY);            // Clear left most circle
  display.fillCircle(x + scale * 3, y, scale - linesize,EPD_GREY);            // Clear right most circle
  display.fillCircle(x - scale, y - scale, scale * 1.4 - linesize,EPD_GREY);  // left middle upper circle
  display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize,EPD_GREY); // Right middle upper circle
  display.fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9, scale * 2 - linesize * 2 + 2,EPD_GREY); // Upper and lower lines
  
}
//#########################################################################################
void addrain(int x, int y, int scale) {
  y = y + scale / 2;
  for (int i = 0; i < 6; i++) {
    display.drawLine(x - scale * 4 + scale * i * 1.3 + 0, y + scale * 1.9, x - scale * 3.5 + scale * i * 1.3 + 0, y + scale,EPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 4 + scale * i * 1.3 + 1, y + scale * 1.9, x - scale * 3.5 + scale * i * 1.3 + 1, y + scale,EPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.3 + 2, y + scale * 1.9, x - scale * 3.5 + scale * i * 1.3 + 2, y + scale,EPD_BLACK);
    }
  }
}
//#########################################################################################
void addsnow(int x, int y, int scale) {
  int dxo, dyo, dxi, dyi;
  for (int flakes = 0; flakes < 5; flakes++) {
    for (int i = 0; i < 360; i = i + 45) {
      dxo = 0.5 * scale * cos((i - 90) * 3.14 / 180); dxi = dxo * 0.1;
      dyo = 0.5 * scale * sin((i - 90) * 3.14 / 180); dyi = dyo * 0.1;
      display.drawLine(dxo + x + 0 + flakes * 1.5 * scale - scale * 3, dyo + y + scale * 2, dxi + x + 0 + flakes * 1.5 * scale - scale * 3, dyi + y + scale * 2,EPD_BLACK);
    }
  }
}
//#########################################################################################
void addtstorm(int x, int y, int scale) {
  y = y + scale / 2;
  for (int i = 0; i < 5; i++) {
    display.drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0, y + scale,EPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1, y + scale,EPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2, y + scale,EPD_BLACK);
    }
    display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0,EPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1,EPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2,EPD_BLACK);
    }
    display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5,EPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5,EPD_BLACK);
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5,EPD_BLACK);
    }
  }
}
//#########################################################################################
void addsun(int x, int y, int scale) {
  int linesize = 3;
  if (scale == Small) linesize = 1;
  int dxo, dyo, dxi, dyi;
  display.fillCircle(x, y, scale,EPD_BLACK);
  display.fillCircle(x, y, scale - linesize,EPD_WHITE);
  for (float i = 0; i < 360; i = i + 45) {
    dxo = 2.2 * scale * cos((i - 90) * 3.14 / 180); dxi = dxo * 0.6;
    dyo = 2.2 * scale * sin((i - 90) * 3.14 / 180); dyi = dyo * 0.6;
    if (i == 0   || i == 180) {
      display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y,EPD_BLACK);
      if (scale != Small) {
        display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y,EPD_BLACK);
        display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y,EPD_BLACK);
      }
    }
    if (i == 90  || i == 270) {
      display.drawLine(dxo + x, dyo + y - 1, dxi + x, dyi + y - 1,EPD_BLACK);
      if (scale != Small) {
        display.drawLine(dxo + x, dyo + y + 0, dxi + x, dyi + y + 0,EPD_BLACK);
        display.drawLine(dxo + x, dyo + y + 1, dxi + x, dyi + y + 1,EPD_BLACK);
      }
    }
    if (i == 45  || i == 135 || i == 225 || i == 315) {
      display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y,EPD_BLACK);
      if (scale != Small) {
        display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y,EPD_BLACK);
        display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y,EPD_BLACK);
      }
    }
  }
}
//#########################################################################################
void addfog(int x, int y, int scale, int linesize) {
  if (scale == Small) y -= 10;
  if (scale == Small) linesize = 1;
  for (int i = 0; i < 6; i++) {
    display.fillRect(x - scale * 3, y + scale * 1.5, scale * 6, linesize,EPD_BLACK);
    display.fillRect(x - scale * 3, y + scale * 2.0, scale * 6, linesize,EPD_BLACK);
    display.fillRect(x - scale * 3, y + scale * 2.7, scale * 6, linesize,EPD_BLACK);
  }
}
//#########################################################################################
void MostlyCloudy(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  addcloud(x, y, scale, linesize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale);
  addcloud(x, y, scale, linesize);
}
//#########################################################################################
void MostlySunny(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  addcloud(x, y, scale, linesize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale);
}
//#########################################################################################
void Rain(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  addcloud(x, y, scale, linesize);
  addrain(x, y, scale);
}
//#########################################################################################
void Cloudy(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) {
    if (IconName.endsWith("n")) addmoon(x,y,scale);
    linesize = 1;
    addcloud(x, y, scale, linesize);
  }
  else {
    y += 25;
    if (IconName.endsWith("n")) addmoon(x,y-15,scale);
    addcloud(x+30, y-35, 4, linesize); // Cloud top right
    addcloud(x-20, y-25, 6, linesize); // Cloud top left
    addcloud(x, y, scale, linesize);   // Main cloud
  }
}
//#########################################################################################
void Sunny(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  scale = scale * 1.5;
  addsun(x, y, scale);
}
//#########################################################################################
void ExpectRain(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  addsun(x - scale * 1.8, y - scale * 1.8, scale);
  addcloud(x, y, scale, linesize);
  addrain(x, y, scale);
}
//#########################################################################################
void ChanceRain(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  addsun(x - scale * 1.8, y - scale * 1.8, scale);
  addcloud(x, y, scale, linesize);
  addrain(x, y, scale);
}
//#########################################################################################
void Tstorms(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  addcloud(x, y, scale, linesize);
  addtstorm(x, y, scale);
}
//#########################################################################################
void Snow(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  addcloud(x, y, scale, linesize);
  addsnow(x, y, scale);
}
//#########################################################################################
void Fog(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  addcloud(x, y, scale, linesize);
  addfog(x, y, scale, linesize);
}
//#########################################################################################
void Haze(int x, int y, bool LargeSize, String IconName) {
  int scale = Small;
  if (LargeSize) scale = Large;
  int linesize = 3;
  if (scale == Small) linesize = 1;
  if (IconName.endsWith("n")) addmoon(x,y,scale);
  addsun(x, y, scale*1.4);
  addfog(x, y, scale*1.4, linesize);
}
//#########################################################################################
void addmoon (int x, int y, int scale){
  if (scale == Large) {
    display.fillCircle(x-37,y-33,scale,EPD_BLACK);
    display.fillCircle(x-27,y-33,scale*1.6,EPD_WHITE);
  }
  else
  {
    display.fillCircle(x-20,y-15,scale,EPD_BLACK);
    display.fillCircle(x-15,y-15,scale*1.6,EPD_WHITE);
  }
}
//#########################################################################################
void Nodata(int x, int y, bool LargeSize) {
  int scale = Small;
  if (LargeSize) scale = Large;
  if (scale == Large) display.setFont(&FreeSans24pt7b); else display.setFont(&FreeSans18pt7b);
  drawString(x, y-10, "N/A",true);
}
//#########################################################################################
void DrawBattery(int x, int y) {
  uint8_t percentage = 100;
  //float voltage = analogRead(35) / 4096.0 * 7.485;
  double voltage = display.readBattery();
  if (voltage > 1) {
    if (voltage > 4.21) percentage = 100;
    else if (voltage < 3.20) percentage = 0;
    else percentage = (voltage - 3.20) * 100 / (4.21 - 3.20);
    display.setFont(&FreeSans9pt7b);
    drawString(x - 35, y, String(voltage, 2) + "V",true);
    display.drawRect(x - 22, y + 2, 19, 10,EPD_BLACK);
    display.fillRect(x - 2, y + 4, 3, 5,EPD_BLACK);
    display.fillRect(x - 20, y + 4, 17 * percentage / 100.0, 6,EPD_BLACK);
  }
}
//#########################################################################################
/* (C) D L BIRD
    This function will draw a graph on a ePaper/TFT/LCD display using data from an array containing data to be graphed.
    The variable 'max_readings' determines the maximum number of data elements for each array. Call it with the following parametric data:
    x_pos - the x axis top-left position of the graph
    y_pos - the y-axis top-left position of the graph, e.g. 100, 200 would draw the graph 100 pixels along and 200 pixels down from the top-left of the screen
    width - the width of the graph in pixels
    height - height of the graph in pixels
    Y1_Max - sets the scale of plotted data, for example 5000 would scale all data to a Y-axis of 5000 maximum
    data_array1 is parsed by value, externally they can be called anything else, e.g. within the routine it is called data_array1, but externally could be temperature_readings
    auto_scale - a logical value (TRUE or FALSE) that switches the Y-axis autoscale On or Off
    barchart_on - a logical value (TRUE or FALSE) that switches the drawing mode between barhcart and line graph
    barchart_colour - a sets the title and graph plotting colour
    If called with Y!_Max value of 500 and the data never goes above 500, then autoscale will retain a 0-500 Y scale, if on, the scale increases/decreases to match the data.
    auto_scale_margin, e.g. if set to 1000 then autoscale increments the scale by 1000 steps.
*/
void DrawGraph(int x_pos, int y_pos, int gwidth, int gheight, float Y1Min, float Y1Max, String title, float DataArray[], int readings, boolean auto_scale, boolean barchart_mode) {
#define auto_scale_margin 0 // Sets the autoscale increment, so axis steps up in units of e.g. 3
#define y_minor_axis 5      // 5 y-axis division markers
#define OffsetTextY 15
int OffsetText = 25;
  int maxYscale = -10000;
  int minYscale =  10000;
  int last_x, last_y;
  float x1, y1, x2, y2;
  if (auto_scale == true) {
    for (int i = 1; i < readings; i++ ) {
      if (DataArray[i] >= maxYscale) maxYscale = DataArray[i];
      if (DataArray[i] <= minYscale) minYscale = DataArray[i];
    }
    maxYscale = round(maxYscale + auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Max
    Y1Max = round(maxYscale+0.5);
    if (minYscale != 0) minYscale = round(minYscale - auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Min
    Y1Min = round(minYscale);
  }
  // Draw the graph
  last_x = x_pos + 1;
  last_y = y_pos + (Y1Max - constrain(DataArray[1], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight;
  display.drawRect(x_pos, y_pos, gwidth + 3, gheight + 2,EPD_BLACK);
  //display.setFont(ArialMT_Plain_10);

  display.setFont(&FreeSans9pt7b);
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
  //drawString(x_pos + gwidth / 2-OffsetText, y_pos - 10, title);
  drawString(x_pos, y_pos - 10, title,true);
  display.setFont(&FreeSans9pt7b);
  //display.setTextAlignment(TEXT_ALIGN_RIGHT);
  // Draw the data
  for (int gx = 1; gx < readings; gx++) {
      x1 = last_x;
      y1 = last_y;
      x2 = x_pos + gx * gwidth/(readings-1)-1 ; // max_readings is the global variable that sets the maximum data that can be plotted
      y2 = y_pos + (Y1Max - constrain(DataArray[gx], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight + 1;
      if (barchart_mode) {
        display.fillRect(x2, y2, (gwidth/readings)-1, y_pos + gheight - y2 + 1,EPD_BLACK);
      } else {
        display.drawLine(last_x, last_y, x2, y2,EPD_BLACK);
        //Double the line width
        display.drawLine(last_x+1, last_y, x2+1, y2,EPD_BLACK);
        display.drawLine(last_x, last_y+1, x2, y2+1,EPD_BLACK);
        display.drawLine(last_x+1, last_y+1, x2+1, y2+1,EPD_BLACK);
        Serial.print("Graph Data "); Serial.print(DataArray[gx]); Serial.print("   Y "); Serial.println(y2); 
      }
      last_x = x2;
      last_y = y2;
  }

  for (int i = 0; i <= 3; i++) {
    drawString(5 + x_pos + gwidth / 3 * i-5, y_pos + gheight + 3+OffsetTextY, String(i),true);
  }


  drawString(x_pos+gwidth/2+12-OffsetText,y_pos+gheight+18+OffsetTextY,"Days",true);

  //Draw the Y-axis scale
  if(Y1Max>=100) {OffsetText = OffsetText*2;}
  for (int spacing = 0; spacing <= y_minor_axis; spacing++) {
  #define number_of_dashes 20
    for (int j = 0; j < number_of_dashes; j++) { // Draw dashed graph grid lines
      if (spacing < y_minor_axis) display.drawFastHLine((x_pos + 3 + j * gwidth / number_of_dashes), y_pos + (gheight * spacing / y_minor_axis), gwidth / (2 * number_of_dashes),EPD_BLACK);
    }
    if ( (Y1Max-(float)(Y1Max-Y1Min)/y_minor_axis*spacing) < 10) {
      drawString(x_pos-2-OffsetText, y_pos+gheight*spacing/y_minor_axis-5+ OffsetTextY, String((Y1Max-(float)(Y1Max-Y1Min)/y_minor_axis*spacing+0.01), 1),true);}
    else {
      if (Y1Min < 1) drawString(x_pos-OffsetText, y_pos + gheight * spacing / y_minor_axis - 5+OffsetTextY, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing+0.01), 1),true);
      else drawString(x_pos-OffsetText, y_pos + gheight * spacing / y_minor_axis - 5+OffsetTextY, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 0),true); // +0.01 prevents -0.00 occurring
    }
  }
}

//#########################################################################################
void DecodeReadings(byte bitData){
//Read the binary data from the bank and apply conversions where necessary to scale and format data

  dataByte=(dataByte<<1)|bitData;
  nosBits++;
  if (nosBits==8)
  {
    nosBits=0;
    manchester[nosBytes]=dataByte;
    nosBytes++;
    //Serial.print("B");
  }
  if(nosBytes==maxBytes)
  {
    dataByte = 0xFF;
    // Subroutines to extract data from Manchester encoding and error checking
    
    // Identify channels 1 to 8 by looking at 3 bits in byte 3
    int stnId = ((manchester[3]&B01110000)/16)+1;

    Serial.print("433MHz Received Station Id: "); Serial.println(stnId);
    // Identify sensor by looking for sensorID in byte 1 (F007th Ambient Thermo-Hygrometer = 0x45)
    dataType = manchester[1];  
    
    // Gets raw temperature from bytes 3 and 4 (note this is neither C or F but a value from the sensor) 
    Newtemp = (float((manchester[3]&B00000111)*256)+ manchester[4]);
    
    // Gets humidity data from byte 5
    Newhum =(manchester [5]); 
    
    
    if ( Checksum (countof(manchester)-2, manchester+1) == manchester[MAX_BYTES-1])
    {
      // Checks sensor is a F007th with a valid humidity reading equal or less than 100
      if ((dataType == 0x45) && (Newhum <= 100))
      {
        if(stnId==1){
          if(Units == "M") Channel1_Temp = float(Newtemp-720)/18.0; //C
          if(Units != "M") Channel1_Temp = float(Newtemp-400)/10.0; //F
          Channel1_Hum = Newhum;
          Channel1_New = true;
        }
        if(stnId==2){
          if(Units == "M") Channel2_Temp = float(Newtemp-720)/18.0; //C
          if(Units != "M") Channel2_Temp = float(Newtemp-400)/10.0; //F
          Channel2_Hum = Newhum;
          Channel2_New = true;
        }
        if(stnId==3){
          if(Units == "M") Channel3_Temp = float(Newtemp-720)/18.0; //C
          if(Units != "M") Channel3_Temp = float(Newtemp-400)/10.0; //F
          Channel3_Hum = Newhum;
          Channel3_New = true;
        }        
        if(stnId==4){
          if(Units == "M") Channel4_Temp = float(Newtemp-720)/18.0; //C
          if(Units != "M") Channel4_Temp = float(Newtemp-400)/10.0; //F
          Channel4_Hum = Newhum;
          Channel4_New = true;
        }
        if(stnId==5){
          if(Units == "M") Channel5_Temp = float(Newtemp-720)/18.0; //C
          if(Units != "M") Channel5_Temp = float(Newtemp-400)/10.0; //F
          Channel5_Hum = Newhum;
          Channel5_New = true;
        }
        if(stnId==6){
          if(Units == "M") Channel6_Temp = float(Newtemp-720)/18.0; //C
          if(Units != "M") Channel6_Temp = float(Newtemp-400)/10.0; //F
          Channel6_Hum = Newhum;
          Channel6_New = true;
        }        
      }
    }
  }
}


//#########################################################################################

uint8_t Checksum(int length, uint8_t *buff)
{
    uint8_t mask = 0x7C;
    uint8_t checksum = 0x64;
    uint8_t data;
    int byteCnt;  

    for ( byteCnt=0; byteCnt < length; byteCnt++)
    {
      int bitCnt;
  data = buff[byteCnt];
  
  for ( bitCnt= 7; bitCnt >= 0 ; bitCnt-- )
  {
            uint8_t bit;
      
            // Rotate mask right
      bit = mask & 1;
      mask =  (mask >> 1 ) | (mask << 7);
      if ( bit )
      {
    mask ^= 0x18;
      }
      
      // XOR mask into checksum if data bit is 1      
      if ( data & 0x80 )
      {
        checksum ^= mask;
            }
      data <<= 1; 
  }
    }
    return checksum;
}



void drawString(int16_t xMove, int16_t yMove, String strUser, bool bBlack){
  //Write same text
  if(bBlack) { 
    display.setTextColor(EPD_BLACK, EPD_WHITE); //Black text on white background
  } else {
    display.setTextColor(3, EPD_WHITE); //GREY text on white background
  }
    display.setCursor(xMove, yMove);
    display.print(strUser);

}
