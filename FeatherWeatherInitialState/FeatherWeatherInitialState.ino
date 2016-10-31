/*
  Ricardo Mena C
  ricardo@crcibernetica.com
  http://crcibernetica.com

  License
  **********************************************************************************
  This program is free software; you can redistribute it 
  and/or modify it under the terms of the GNU General    
  Public License as published by the Free Software       
  Foundation; either version 3 of the License, or        
  (at your option) any later version.                    
                                                        
  This program is distributed in the hope that it will   
  be useful, but WITHOUT ANY WARRANTY; without even the  
  implied warranty of MERCHANTABILITY or FITNESS FOR A   
  PARTICULAR PURPOSE. See the GNU General Public        
  License for more details.                              
                                                        
  You should have received a copy of the GNU General    
  Public License along with this program.
  If not, see <http://www.gnu.org/licenses/>.
                                                        
  Licence can be viewed at                               
  http://www.gnu.org/licenses/gpl-3.0.txt

  Please maintain this license information along with authorship
  and copyright notices in any redistribution of this code
  **********************************************************************************
  */

/*
Conecctions
  Feather MO|
 Pinout     |  FONA
------------------
  0          TX
  1          RX
  5          Rst
  6          Key
  9          PStat
  10         Vio
  BAT        Bat
  ----------------
            | Weather Shield
  11         Rain
  12         Wind Speed
  A0         Wind Direction
  3v3        Weather Shield (5v)
  ----------------
            | Soil Sensor
  A2         Clock
  A3         Data
  ----------------
  20         SDA
  21         SCL
  ----------------
  USB        Solar Panel input  
------------------
  
*/

#include <Adafruit_SleepyDog.h>
#include "Adafruit_FONA.h"
//#include "Adafruit_MQTT.h"//??
//#include "Adafruit_MQTT_FONA.h"//??
//#include <Wire.h>//??
//#include <SPI.h>
//#include <SHT1x.h>//??
#include "Adafruit_SI1145.h"
#include <SparkFunBME280.h>

#undef min
#undef max

#define serial Serial
#define fonaSerial Serial1
//#define _dbg  debug

#define SERIAL_BAUD   115200
#define DEBUG //uncoment for debuging

//-----------------Emoncms Configurations---------
#define SERVER "insecure-groker.initialstate.com"
#define ACCESS_KEY "LDwRk4tD7et0n7Y6f14POvomdARY7BmD"
#define BUCKET_KEY "AKB33R64RG4R"


//---------FONA network-------------
#define FONA_RX 1
#define FONA_TX 0
#define FONA_RST 5
#define FONA_KEY 6
#define FONA_PS 9
#define FONA_VIO 10
//-----------------------------------

//-----------------------------------
#define WSPEED 11
#define RAIN 12
#define WDIR A0 
//-----------------------------------

//----------SHT10 Sensor-------------
/*
#define SHT_DATA A2
#define SHT_CLOCK A3
SHT1x sht1x(SHT_DATA, SHT_CLOCK);//Soil
*/
//-----------------------------------

//----------SI1145 Sensor------------
Adafruit_SI1145 uv = Adafruit_SI1145();//UV Light
boolean uvInitialization = false;
//-----------------------------------

//----------BME280 Sensor------------
BME280 weatherSensor;
float bmpTemperature;
float bmpPressure;
float bmpAltitude;
float bmpHumidity;
boolean bmpInitialization = false;
//-----------------------------------

//-----------------FONA things--------------------
// this is a large buffer for replies
char replybuffer[255];

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

// Size of the geo fence (in meters)
const float maxDistance = 100;

//Last Fona Battery percentage
uint16_t last_fona_battery_state;

// Initial GPS read
//uint8_t gps_fix = 0;

// Latitude & longitude for distance measurement
float initialLatitude, initialLongitude, latitude, longitude, speed_kph, heading, altitude, distance;
float gsm_latitude, gsm_longitude;

//boolean lock_initial_gps = false;
//uint8_t gps_fix_failures = 0;

// FONA GPRS configuration
#define FONA_APN             ""  // APN used by cell data service (leave blank if unused)
#define FONA_USERNAME        ""  // Username used by cell data service (leave blank if unused).
#define FONA_PASSWORD        ""  // Password used by cell data service (leave blank if unused).

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;
volatile float dailyrainin = 0; // [rain inches so far today in local time]
float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min

long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

byte windspdavg[120]; //120 bytes to keep track of 2 minute average
int winddiravg[1200]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]

#define MAX_TX_FAILURES      4  // Maximum number of publish failures in a row before resetting the whole sketch.

uint8_t txFailures = 0;                                       // Count of how many publish failures have occured in a row.

//------------------------------------------------

uint32_t previousMillis_1 = 0;
uint32_t previousMillis_2 = 0;
uint32_t time_on = 1000;//
uint32_t timer_on_fona_check = 1000;

//======================Default Functions=============================

void setup() {
  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor  
  pinMode(FONA_KEY, OUTPUT);
  pinMode(FONA_PS, INPUT);
  pinMode(FONA_VIO, OUTPUT);
  digitalWrite(FONA_VIO, HIGH);

  seconds = 0;
  lastSecond = millis();
  
  #if defined(DEBUG)
    serial.begin(SERIAL_BAUD);
  #endif
  delay(5000);

  //Start up FONA in setup method;
  init_fona();
  print_IMEI();

  //Timers uptade
  time_on *= 60*2;//Send messages every 5 min aprox.
  timer_on_fona_check *= 30;//Check FONA every 30s aprox;

  //Weather Interrupts
  attachInterrupt(RAIN, rainIRQ, FALLING);
  attachInterrupt(WSPEED, wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();  


  //---SI1145---
  if (! uv.begin()) {
    #if defined(DEBUG)
    Serial.println("Didn't find Si1145");
    #endif
  }else{
    uvInitialization = true;
    #if defined(DEBUG)
      Serial.println("Si1145 initialized");
    #endif
  }//end if   
  //------

  bmpInitialization = BME280Initialization();
  
}//end setup

void loop(){
  uint32_t m_millis = millis();
  uint32_t c_millis = millis();
  temporizer_1(m_millis, time_on);
  check_connection(c_millis, timer_on_fona_check);//Check connections
  windIterate();
  #if defined(DEBUG)
  serial.flush();
  #endif
}//end loop

//====================================================================

//=======================Fona Related Functions=======================

void init_fona(){
  fona_off();
  delay(2000);
  fona_on();
  delay(2000);
  fonaSerial.begin(4800);
  if(!check_fona()){// See if the FONA is responding
    halt(F("Couldn't find FONA"));
  }//end if
  
  //APN configuration
  //fona.setGPRSNetworkSettings(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD));
  fona.setGPRSNetworkSettings(F("kolbi3g"), F(""), F(""));

  #if defined(DEBUG)
  serial.println(F("Waiting 20s.."));
  #endif
  
  delay(20000);//Wait for FONA

  gprs_disable();
  gprs_enable(0);
  fona_gps_on();
  gpFIX();

}//end init_fona

uint8_t check_fona(){
  // See if the FONA is responding
  if (!fona.begin(fonaSerial)) {
    #if defined(DEBUG)
      serial.println(F("Couldn't find FONA"));
    #endif
    return 0;
  }//end if
  #if defined(DEBUG)
    serial.println(F("FONA is OK"));
  #endif
  return 1;
}//end check_fona

uint8_t check_connection(uint32_t &timer, uint32_t interval){
  //noInterrupts();//<-------?????27/10/2016
  if(timer - previousMillis_2 > interval) {
    serial.println(F("Checking FONA connection"));
    if((!fona.GPRSstate())||(fona.getNetworkStatus() != 1)){
      //halt(F("Network connection problems, resetting..."));
      serial.println(F("Network connection problems, resetting..."));
      delay(1000);
      init_fona();
    }else if ((txFailures >= MAX_TX_FAILURES)) {//!fona.TCPconnected() || (txFailures >= MAX_TX_FAILURES)
      serial.println(F("Connection failed, resetting..."));
      delay(1000);
      //halt(F("Connection failed, resetting..."));
      init_fona();
    }//end if
      serial.println(F("FONA connection state : ok"));
      previousMillis_2 = timer;
  }//end if
  //interrupts();//<-------?????27/10/2016
}//end check_connection

void fona_on(){
  #if defined(DEBUG)
    serial.println("Turning on Fona: ");
  #endif
  while(digitalRead(FONA_PS)==LOW){
    digitalWrite(FONA_KEY, LOW);
    delay(3000);
    break;    
  }//end while
  digitalWrite(FONA_KEY, HIGH);
  //delay(2000); 
}//end fona_on

void fona_off(){
  #if defined(DEBUG)
    serial.println("Turning off Fona: ");
  #endif
  while(digitalRead(FONA_PS)==HIGH){
    digitalWrite(FONA_KEY, LOW);
    delay(3000);
    break;
  }//end while
  digitalWrite(FONA_KEY, HIGH);
  //delay(4000);
}//end fona_off

int gprs_enable(int maxtry){
  // turn GPRS on
  if (!fona.enableGPRS(true)){
    #if defined(DEBUG)
      serial.print(F("Failed to turn on GPRS = "));
      serial.println(maxtry);
    #endif
    if(maxtry > 200){
      halt(F("GPRS Failed, shutdown"));
    }
    maxtry +=1;
    gprs_enable(maxtry);
  }else{
    #if defined(DEBUG)
      serial.println(F("GPRS ON"));
    #endif
  }//end if
}//end gprs_enable

int gprs_disable(){
  // turn GPRS off
  if (!fona.enableGPRS(false)){
    #if defined(DEBUG)
      serial.println(F("Failed to turn GPRS off"));
    #endif
    return 0;
  }else{
    #if defined(DEBUG)
      serial.println(F("GPRS OFF"));
    #endif
    return 1;
  }//end if
}//end gprs_disable

int fona_gps_on(void){                    //turn GPS on
  if (!fona.enableGPS(true)){
    return 0;                             //Failed to turn GPS on
  }//end if
  delay(2500);
  return 1;
}//end fona_gps_on

int fona_gps_off(void){                    // turn GPS off
  delay(2500);
  if (!fona.enableGPS(false)){
    return 0;                              //Failed to turn GPS off
  }//end if
  return 1;
}//end fona_gps_off

uint8_t fona_gps_location(){
  // check for GPS location, Latitude & longitude for distance measurement
  uint8_t gps_fix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  return gps_fix;
}//end fona_gps_location

uint16_t fona_get_battery(void){
  // Grab battery reading
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  return vbat;
}//end fona_get_battery

//====================================================================

//======================Miscelanius Funcions==========================

void print_IMEI(void){
  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
  #if defined(DEBUG)
    #if defined(DEBUG)
    serial.print("SIM card IMEI: "); serial.println(imei);
    #endif
  #endif
  }//end if  
}//end print_IMEI

//float gpslat = 9.9381179;
//float gpslong = -84.0957641;

void temporizer_1(uint32_t &timer, uint32_t interval){
  if((timer - previousMillis_1) > interval) {
    //noInterrupts();//<-------?????27/10/2016
  //---------Temporized code here------------
//  check_connection();
  BME280Read();
  
  char GPSlocation[300];
  if(fona_gps_location()){
      sprintf(GPSlocation, "&gpsweather=%s,%s", float_to_string(latitude, 7).c_str(), float_to_string(longitude, 7).c_str());
  }else{
    sprintf(GPSlocation, "");//
    //sprintf(GPSlocation, "&gpsweather=%s,%s", float_to_string(gpslat, 7).c_str(), float_to_string(gpslong, 7).c_str());//
    //gpslat += 0.00001;
    //gpslong += 0.00001;
  }//end if

  if(GPSlocation != ""){
    if(send_url(String(GPSlocation)) == -1){
      serial.println("Problem sending GPS location data");
      txFailures++;
    }else{
      txFailures = 0;  
    }//end if    
  }//end if

  char BME280Data[300];
  BME280Read();
  if(bmpInitialization){
    sprintf(BME280Data, "&temperature=%s&humidity=%s&pressure=%s&bmaltitude=%s", float_to_string(bmpTemperature, 2).c_str(), float_to_string(bmpHumidity, 2).c_str(), float_to_string(bmpPressure, 2).c_str(), float_to_string(bmpAltitude, 2).c_str());
  }else{
    sprintf(BME280Data, "&temperature=%d&humidity=%d&pressure=%d&bmaltitude=%d", -1, -1, -1, -1);
  }//end if

  if(send_url(String(BME280Data)) == -1){
    serial.println("Problem sending BME280 data");
    txFailures++;
  }else{
    txFailures = 0;  
  }//end if    
  
  calcWeather();
  char weatherConditions[300];
  sprintf(weatherConditions, "&wind=%s&windSpeed=%s&rainin=%s&dailyrainin=%s", String(winddir_avg2m).c_str(), float_to_string(windspdmph_avg2m, 2).c_str(), float_to_string(rainin, 2).c_str(), float_to_string(dailyrainin, 2).c_str());

  if(send_url(String(weatherConditions)) == -1){
    serial.println("Problem sending Weather Conditions");
    txFailures++;
  }else{
    txFailures = 0;  
  }//end if    
  
  String testMessage = "&battery="+String(fona_get_battery()) + lightSensorRead();
  if(send_url(testMessage) == -1){
    serial.println("Problem sending Battery State");
    txFailures++;
  }else{
    txFailures = 0;  
  }//end if    
  //------------------------------
    previousMillis_1 = timer;
  }//end if
  //interrupts();//<-------?????27/10/2016
}//end temporizer

void gpFIX(){
  // Initial GPS read
  bool gpsFix = false;
  if(fona.GPSstatus() >= 2){
    gpsFix = true;
  }
  //do{
  while(!gpsFix){
    //gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
    gpsFix = fona_gps_location();
    //initialLatitude = latitude;
    //initialLongitude = longitude;
    serial.print(F("Waiting GPS fix "));
    serial.println(txFailures);
    delay(200);
    if(txFailures > 200){
      //halt(F("GPS not fix"));
      break;
    }//end if
    txFailures++;
//    return gpsFix;
  //}while(!gpsFix);
  }//end if
  serial.println(F("Can't fix GPS"));
  txFailures = 0; 
//  return 0; 
}//end gpFIX

String float_to_string(float value, uint8_t places) {//Adafruit funtion
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  //int i;
  float tempfloat = value;
  String float_obj = "";

    // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0){
    d *= -1.0;
  }
  // divide by ten for each decimal place
  for (uint8_t i = 0; i < places; i++){
    d/= 10.0;
  }
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0){
    tempfloat *= -1.0;
  }
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }
  // write out the negative if needed
  if (value < 0){
    float_obj += "-";
  }//en if
  
  if (tenscount == 0){
    float_obj += String(0, DEC);
  }//en if
  
  for (uint8_t i = 0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    float_obj += String(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }//en for

  // if no places after decimal, stop now and return
  if (places <= 0){
    return float_obj;
  }//end if

  // otherwise, write the point and continue on
  float_obj += ".";

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (uint8_t i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    float_obj += String(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }//end for
  return float_obj;
}//end float_to_string

String json_split(String &message, String &node_id){
  String brak = "\{";
  String number = "";
  uint16_t j = 0;
  uint8_t data_num = 1;
  node_id = "";
  for(uint8_t i = 0; i < message.length(); i++){
    if((message[i] == ';')||(message[i] == ' ')){
      j = i+1;
      break;
    }else{
      node_id += message[i];
    }//end if
  }//end for

  for(j; j < message.length(); j++){
    if((message[j] == ';')||(message[j] == ' ')){
      brak += String(data_num) + "\:" + number + "\,";
      data_num++;
      //serial.println(brak);
      number = "";
    }else{
      if(message[j]!= '\r'){
        number += message[j];
      }
    }//end if
    
  }//end for
  brak += String(data_num) + "\:" + number+"\}";
  return brak;
  
}//end json_pck

String lightSensorRead(){
  String ligthSensor = "";
  if(uvInitialization){
    //Read SI1145 UV Light sensor
    ligthSensor += "&visible=";
    ligthSensor += uv.readVisible();
    ligthSensor += "&ir=";
    ligthSensor += uv.readIR();
    ligthSensor += "&uv=";
    float UVindex = uv.readUV();
    UVindex /= 100.0; 
    ligthSensor += UVindex;//uv.readUV();  
  }//end if
  return ligthSensor;
}//end lightSensorRead

boolean BME280Initialization(){
  //---BME280---
  weatherSensor.settings.commInterface = I2C_MODE;
  weatherSensor.settings.I2CAddress = 0x77;
  //***Operation settings*****************************//
  weatherSensor.settings.runMode = 3; //  3, Normal mode
  weatherSensor.settings.tStandby = 0; //  0, 0.5ms
  weatherSensor.settings.filter = 0; //  0, filter off
  weatherSensor.settings.tempOverSample = 1;
  weatherSensor.settings.pressOverSample = 1;
  weatherSensor.settings.humidOverSample = 1;
  weatherSensor.begin();
  //---------
  return true;
}//end BME280Initialization

void BME280Read(){
  bmpTemperature = weatherSensor.readTempC();
  bmpPressure =weatherSensor.readFloatPressure();
  bmpAltitude = weatherSensor.readFloatAltitudeMeters();
  bmpHumidity = weatherSensor.readFloatHumidity();
}//end bmp085Read

void windIterate(){
  //Keep track of which minute it is
  if(millis() - lastSecond >= 1000){
    
    lastSecond += 1000;

    //Take a speed and direction reading every second for 2 minute average
    if(++seconds_2m > 119) seconds_2m = 0;

    //Calc the wind speed and direction every second for 120 second to get 2 minute average
    float currentSpeed = windSpeed();
    
    //float currentSpeed = random(5); //For testing
    int adc = 0;
    int currentDirection = windDirection(adc);
    windspdavg[seconds_2m] = (int)currentSpeed;
    winddiravg[seconds_2m] = currentDirection;

    //Check to see if this is a gust for the minute
    if(currentSpeed > windgust_10m[minutes_10m]){
      windgust_10m[minutes_10m] = currentSpeed;
      windgustdirection_10m[minutes_10m] = currentDirection;
    }

    //Check to see if this is a gust for the day
    if(currentSpeed > windgustmph){
      windgustmph = currentSpeed;
      windgustdir = currentDirection;
    }
    if(++seconds > 59){
      seconds = 0;

      if(++minutes > 59) minutes = 0;
      if(++minutes_10m > 9) minutes_10m = 0;
    }//end if
  }
}//end windIterate

//Returns the instataneous wind speed
float windSpeed(){
  float deltaTime = millis() - lastWindCheck; //750ms
  deltaTime /= 1000.0; //Covert to seconds
  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4
  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();
  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH
  return(windSpeed);
}//end windSpeed

//Read the wind direction sensor, return heading in degrees
int windDirection(int &ad){
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor
  ad = adc;
  //ad = adc;
  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.
 
  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (271); // error, disconnected?
 
}//end windDirection

void calcWeather(){
  //Calculates each of the variables that wunderground is expecting
  //Calc winddir
  int ac = 0;
  winddir = windDirection(ac);
  //Calc windspeed
  windspeedmph = windSpeed();

  //Calc windgustmph
  //Calc windgustdir
  //Report the largest windgust today
  windgustmph = 0;
  windgustdir = 0;

  //Calc windspdmph_avg2m
  float temp = 0;
  for(int i = 0 ; i < 120 ; i++)
    temp += windspdavg[i];
  temp /= 120.0;
  windspdmph_avg2m = temp;

  //Calc winddir_avg2m
  temp = 0; //Can't use winddir_avg2m because it's an int
  for(int i = 0 ; i < 120 ; i++)
    temp += winddiravg[i];
  temp /= 120;
  winddir_avg2m = temp;

  //Total rainfall for the day is calculated within the interrupt
  //Calculate amount of rainfall for the last 60 minutes
  rainin = 0;  
  for(int i = 0 ; i < 60 ; i++)
    rainin += rainHour[i];
}//end calcWeather

void flushSerial() {
  #if defined(DEBUG)
  while (serial.available()) 
    serial.read();
  #endif
}//end flushSerial

void halt(const __FlashStringHelper *error) {
  serial.println(error);
  delay(1000);
  Watchdog.enable(1000);
  Watchdog.reset();
  while (1) {}
}//end halt
//====================================================================

//==========================Web Functions=============================
// Post data to website
int send_url(String raw_paq){
  uint16_t statuscode;
  int16_t length;
  String node = "";//Store node id
  //String json = json_split(raw_paq, node);//split packet into json format and store node id througth reference
  String url = "GET ";
  url += "/api/events?accessKey="+String(ACCESS_KEY)+"&bucketKey="+String(BUCKET_KEY);

//  int l_url = url.length();//strlen(data)

  char c_url[800];
  sprintf(c_url, "%s%s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", url.c_str(), raw_paq.c_str(), SERVER);

  // if there's a successful connection:
  if (fona.TCPconnect(SERVER, 80)) {
    Serial.println("connecting...");
    // Make a HTTP request:
    
    if(fona.TCPconnected()){
        Serial.println(F("Sending"));
        Serial.println(c_url);
        fona.TCPsend(c_url, strlen(c_url));
    }else {
      // if you couldn't make a connection:
      Serial.println("ERROR connecting");
    }
  }

  fona.TCPclose();
  //flushSerial();
  
 /* #if defined(DEBUG)  
    serial.println(c_url);
    serial.println(F("****"));
  #endif

  char data[100];
  sprintf(data, "[{\"key\":\"temperaute\",\"value\":\"%d\"}]", 1);
  char x_url[800];
  sprintf(x_url, "%s", url.c_str());
  //if (!fona.HTTP_GET_start(c_url, &statuscode, (uint16_t *)&length)) {
  if (!fona.HTTP_POST_start(x_url, F("application/json"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
    #if defined(DEBUG)
    serial.println("GPRS send failed!");
    #endif
    return -1;
  }else{
    #if defined(DEBUG)
    serial.println("GPRS send ok");
    #endif
    #if defined(FREERAM)
        serial.print("Free RAM TOP = ");
        serial.println(freeRam());
    #endif    
  }
  while (length > 0) {
    while (fona.available()) {
      char c = fona.read();     
    // Serial.write is too slow, we'll write directly to Serial register!
      #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
          loop_until_bit_is_set(UCSR0A, UDRE0); // Wait until data register empty. 
          UDR0 = c;
      #else
          #if defined(DEBUG)
          serial.write(c);
          #endif
      #endif
      length--;
      if (! length) break;
    }//end while
  }//end while
  #if defined(DEBUG)
    serial.println(F("\n****"));
  #endif
  fona.HTTP_POST_end();*/
}//end send_url

//====================================================================

//==========================Interrupt routines========================
void rainIRQ(){
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2

  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  if (raininterval > 10){ // ignore switch-bounce glitches less than 10mS after initial edge
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }//end if
}//end rainIRQ

void wspeedIRQ(){// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
  if (millis() - lastWindIRQ > 10){ // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }//end if
}//end wspeedIRQ
//====================================================================


