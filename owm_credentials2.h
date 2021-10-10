// Change to your WiFi credentials
String City = "";

char* ssid1     = "Your WiFi SSID 1";      // For your network
char* password1 = "Your WiFi Password 1";  // For your network
String City1          = "Horsham";                      // Your home city See: http://bulk.openweathermap.org/sample/

char* ssid2     = "Your WiFi SSID 2";      // For your network
char* password2 = "Your WiFi Password 2";  // For your network
String City2          = "Haywards Heath";                      // Your home city See: http://bulk.openweathermap.org/sample/

//Labels for on screen temperature readings
//Set Senders to Channels 1, 2 & 3 for Label 1, 2 & 3
String Label1         = "Inside Van";
String Label2         = "Underfloor Water";
String Label3         = "Outdoor";



// Use your own API key by signing up for a free developer account at https://openweathermap.org/
String apikey       = "Your API Key";                   // See: https://openweathermap.org/
const char server[] = "api.openweathermap.org";


//Set your location according to OWM locations
String Country       = "GB";                            // Your country  
String Language      = "EN";                            // NOTE: Only the weather description (not used) is translated by OWM
                                                        // Arabic (AR) Czech (CZ) English (EN) Greek (EL) Persian(Farsi) (FA) Galician (GL) Hungarian (HU) Japanese (JA)
                                                        // Korean (KR) Latvian (LA) Lithuanian (LT) Macedonian (MK) Slovak (SK) Slovenian (SL) Vietnamese (VI)
String Hemisphere    = "north";                         // or "south"  
String Units         = "M";                             // Use 'M' for Metric or I for Imperial 
const char* Timezone = "GMT0BST,M3.5.0/01,M10.5.0/02";  // Choose your time zone from: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv 
// Example time zones
//const char* Timezone = "MET-1METDST,M3.5.0/01,M10.5.0/02"; // Most of Europe
//const char* Timezone = "CET-1CEST,M3.5.0,M10.5.0/3";       // Central Europe
//const char* Timezone = "EST-1METDST,M3.5.0/01,M10.5.0/02"; // Most of Europe
//const char* Timezone = "EST5EDT,M3.2.0,M11.1.0";           // EST USA  
//const char* Timezone = "CST6CDT,M3.2.0,M11.1.0";           // CST USA
//const char* Timezone = "MST7MDT,M4.1.0,M10.5.0";           // MST USA
//const char* Timezone = "NZST-12NZDT,M9.5.0,M4.1.0/3";      // Auckland
//const char* Timezone = "EET-2EEST,M3.5.5/0,M10.5.5/0";     // Asia
//const char* Timezone = "ACST-9:30ACDT,M10.1.0,M4.1.0/3":   // Australia
