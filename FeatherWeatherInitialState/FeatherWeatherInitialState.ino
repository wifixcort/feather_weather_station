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
  USB        Solar Panel (+) input  
------------------
  
*/
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <Adafruit_SleepyDog.h>
#include "Adafruit_FONA.h"
#include "Adafruit_SI1145.h"
#include <SparkFunBME280.h>
#include <RTCZero.h>


#undef min
#undef max

#define serial Serial
#define fonaSerial Serial1
//#define _dbg  debug

#define SERIAL_BAUD   115200
#define DEBUG //uncoment for debuging

//----------InitialState Configurations---------
#define SERVER "insecure-groker.initialstate.com"
#define ACCESS_KEY ""
#define BUCKET_KEY ""


//---------FONA Pinout Connections---
#define FONA_RX 1
#define FONA_TX 0
#define FONA_RST 5
#define FONA_KEY 6
#define FONA_PS 9
#define FONA_VIO 10
//-----------------------------------

//---------Weather Meters Pinout-----
#define WSPEED 11
#define RAIN 12
#define WDIR A0 
//-----------------------------------

//----------SI1145 Sensor------------
Adafruit_SI1145 light_sensor = Adafruit_SI1145();//UV Light
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

//Last Fona Battery percentage
uint16_t last_fona_battery_state;

// Latitude & longitude for distance measurement
float initialLatitude, initialLongitude, latitude, longitude, speed_kph, heading, altitude, distance;

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;
volatile float dailyrainin = 0; // [rain inches so far today in local time]
float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min

long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
//byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

long lastWindCheck = 0;

/*
 * Why use volatile Variables for interrupt routines
 * Typically global variables are used to pass data between an ISR and the main program. 
 * To make sure variables shared between an ISR and the main program are updated correctly, declare them as volatile.
 * If they are not declared as "volatile" the compiler could optimize the code, blocking unexpected changes(like an interrups
 * modifying that variable), causing undesirable results
*/
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

/*----We need to keep track of the following variables:
 * Wind speed/dir each update (no storage)
 * Wind gust/dir over the day (no storage)
 * Wind speed/dir, avg over 2 minutes (store 1 per second)
 * Rain over the past hour (store 1 per minute)
 * Total rain over date (store one per day)
*/

byte windspdavg[120]; //120 bytes to keep track of 2 minute average
int winddiravg[120]; //120 ints to keep track of 2 minute average
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]


#define MAX_TX_FAILURES      4  // Maximum number of publish failures in a row before resetting the whole sketch.

uint8_t txFailures = 0;//Count of how many publish failures have occured in a row.

//------------------------------------------------

RTCZero rtc;/* Create an rtc object */

uint32_t previousMillis_1 = 0;//Save last time when Fona send a message
uint32_t previousMillis_2 = 0;//Save last time when Fona was checked
uint32_t time_on = 1000;//
uint32_t timer_on_fona_check = 1000;

//======================Sercom4 Configurations========================
/*
 * SERCOM is a configurable Serial Comunication Port that can work as
 * I2C, SPI, USART(Serial), LIN Slave. Whe have 6 sercoms in total
 * Here Sercom4 is configured as Serial Port
 * More about SERCOMS:
 * https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/muxing-it-up?view=all
*/
Uart Serial2 (&sercom4, A1, A2, SERCOM_RX_PAD_0, UART_TX_PAD_0);
void SERCOM4_Handler(){
  Serial2.IrqHandler();
}//end SERCOM4_Handler

#define openlog Serial2 //Easy identification for my Log Serial port
//#define serial openlog

//======================Default Functions=============================

void debug_print(const __FlashStringHelper *status, const __FlashStringHelper *log_message, boolean send_to_openlog = false);

const uint8_t LED = 13;
uint32_t pris = 0;  
void blink(uint32_t timer, uint32_t interval){
  if(timer - pris > interval) {
    // save the last time you blinked the LED 
    if(!digitalRead(LED)){
      digitalWrite(LED, HIGH);
    }else{
      digitalWrite(LED, LOW);  
    }//end if
    pris = timer;
  }//end if
}//end temporizer

void setup() {
  
  pinMode(LED, OUTPUT);
  pinMode(WSPEED, INPUT_PULLUP);  // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP);    // input from wind meters rain gauge sensor  
  pinMode(FONA_PS, INPUT);        //Power Supply Pin. It indicates whether FONA is ON or OFF
  pinMode(FONA_KEY, OUTPUT);      //Tie this pin to ground for 2 seconds to turn the module on or off. 
                                  //It's not a level signal so it isn't like "low is off, high is on"
  pinMode(FONA_VIO, OUTPUT);      //Output to stablish Fona Logic
  digitalWrite(FONA_VIO, HIGH);   //Sets the logic level converters to right voltage. In this case 3.3v

  seconds = 0;
  lastSecond = millis();
  
  #if defined(DEBUG)
    serial.begin(SERIAL_BAUD);
  #endif
  delay(5000);

  openlog.begin(9600);
  /*
   * Last Sercom config needed
   * Assign pins A1(TX) & A2(RX) SERCOM functionality
  */
  pinPeripheral(A1, PIO_SERCOM_ALT);
  pinPeripheral(A2, PIO_SERCOM_ALT);  

  //Startup FONA in setup method;
  if(init_fona()){
    restart(F("Serius error reported, restarting"));
  }else{
    debug_print(F("STATUS"),F("FONA configured correctly"));
  }//end if

  
  print_IMEI();

  /*Update default timers configuration*/
  time_on *= 60*2;            //Send HTTP messages to InitialState every 2 min aprox.
  timer_on_fona_check *= 30;  //Check FONA every 30s aprox;

  /*
   * Attaching funcionts to RAIN and WSPEED pins for read wheater meters 
   * More about attachInterrupt().
   * https://www.arduino.cc/en/Reference/attachInterrupt
  */
  attachInterrupt(RAIN, rainIRQ, FALLING);
  attachInterrupt(WSPEED, wspeedIRQ, FALLING);

  interrupts(); // turn on interrupts
  
  if (!light_sensor.begin()) { //Initialize SI1145. This is 
    #if defined(DEBUG)
    debug_print(F("ERROR"), F("Didn't find Si1145"), true);
    #endif
  }else{
    uvInitialization = true;
    #if defined(DEBUG)
      debug_print(F("STATUS"), F("Si1145 initialized"), true);
    #endif
  }//end if   
  //------

  bmpInitialization = BME280Initialization();
  
}//end setup

void loop(){
  uint32_t c_millis = millis();     //Time 
  blink(c_millis, 1000);            //Blink LED 13 every 1s. Indicate that loop is running
  update_initial_state(c_millis, time_on);
  check_connection(c_millis, timer_on_fona_check);//Check FONA connections
  windIterate(); //Keep updated wind averages
  
  #if defined(DEBUG)
  serial.flush();
  #endif
}//end loop

//=======================Fona Related Functions=======================

uint8_t init_fona(){
    
  debug_print(F("STATUS"), F("Turning OFF"), true);
  if(!fona_off()){
    debug_print(F("STATUS"), F("Fona is OFF"), true);
  }//end if
  delay(1000);
  debug_print(F("STATUS"), F("Turning ON "), true);
  if(!fona_on()){
    debug_print(F("STATUS"), F("Fona is ON"), true);
  }//end if
  delay(2000);//Wait until FONA startup his Serial Port

  /*
   * How much time the Feather need to configure FONA
   * it need to be less than 8s
   * 
   * This variable is just for testing
  */
//  uint32_t start_time_measurement = millis();
  
  fonaSerial.begin(4800);
  if(check_fona()){// See if the FONA is responding
    debug_print(F("ERROR"), F("FONA serial port not responding"), true);
    return EXIT_FAILURE;
  }else{
    debug_print(F("STATUS"), F("FONA Serial Port is UP"), true);
  }//end if

  /*=====================APN configuration=====================
   *Some celular providers don't need extra settings 
   *fona.setGPRSNetworkSettings(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD));
  */
  fona.setGPRSNetworkSettings(F("kolbi3g"), F(""), F(""));

  /*During tests I see that is better wait at least 20s
   *Otherwise FONA could not get authenticated on the network cell 
  */
  #if defined(DEBUG)
  debug_print(F("STATUS"), F("Waiting 20s.."));
  #endif
  
  /*
   * This variable is just for testing
  */
//  uint32_t stop_time_measurement = millis();
//  debug_print(F("STATUS"), F("Time invested configuring the FONA 1 = "));
//  serial.println(stop_time_measurement-start_time_measurement);
  
  delay(20000);//Wait for FONA

  /*
   * This variable is just for testing
   * Reset counter
  */
//  start_time_measurement = millis();

  debug_print(F("CHECKING"), F("Network State :"), true);
  if(fona_network_status()){
    debug_print(F("ERROR"), F("Network problem reported"), true);
    return EXIT_FAILURE;
  }else{
    debug_print(F("STATUS"), F("Network OK"), true);
  }

  /*I don't know the GPRS state*/
  debug_print(F("STATUS"), F("Trying to shut off GPRS"), true);
  if(gprs_disable()){
    //If GPRS is off, then it could 
    debug_print(F("WARNING"), F("GPRS disable failed"), true);
  }else{
    debug_print(F("STATUS"), F("GPRS disabled"), true);
  }//end if
  
  debug_print(F("STATUS"), F("Trying to startup GPRS"), true);
  if(gprs_enable()){
    debug_print(F("ERROR"), F("GPRS ON failed"), true);
    return EXIT_FAILURE;
  }else{
    debug_print(F("STATUS"), F("GPRS is ON"), true);
  }//end if

  debug_print(F("STATUS"), F("Synced RTC with NTP servers "), true);
  if(fona_sync_ntp()){
    debug_print(F("WARNING"), F("Failed to enable NTP sync"), true);
  }else{
    char buffer[23];
    fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes! 
    rtc_m0_config(buffer);//SEND BUFFER TO SINCRONIZE RTC
    debug_print(F("STATUS"), F("RTC synchronized"), true);    
  }//end if

  //If GPS could not work
  debug_print(F("STATUS"), F("Trying to startup GPS"), true);
  if(fona_gps_on()){
    debug_print(F("ERROR"), F("GPS ON failed"), true);
    return EXIT_FAILURE;    
  }else{
    debug_print(F("STATUS"), F("GPS is ON"), true);
  }//end if

//  stop_time_measurement = millis();
//  debug_print(F("STATUS"), F("Time invested configuring the FONA 2 = "));
//  serial.println(stop_time_measurement-start_time_measurement);
   
  /*debug_print(F("Waiting GPS first fix"));
  if(fona_gps_fix()){
    debug_print(F("Could not get first fix yet"));
  }else{
    debug_print(F("First Fix succed"));
  }//end if*/
  
  return EXIT_SUCCESS;

}//end init_fona

uint8_t check_fona(){
  // See if the FONA is responding
  if (fona.begin(fonaSerial)) {//Testing FONA serial port
    return EXIT_SUCCESS;
  }//end if
  return EXIT_FAILURE;
}//end check_fona

uint8_t check_connection(uint32_t &timer, uint32_t interval){
  //noInterrupts();//<-------?????27/10/2016
  if(timer - previousMillis_2 > interval) {
    debug_print(F("CHECKING"), F("Network state : "));
    if(fona_network_status()){
      debug_print(F("ERROR"), F("Network problem reported"), true);
      init_fona();
    }else{
      debug_print(F("STATUS"), F("Network ok"), 1);
    }

    debug_print(F("CHECKING"), F("GPRS state : "), true);
    if(!fona.GPRSstate()){
      debug_print(F("ERROR"), F("GPRS problem reported"), true);
      init_fona();
    }else{
      debug_print(F("STATUS"), F("GPRS OK"), true);
    }//end if

    debug_print(F("CHECKING"), F("Transmitions failures limits : "), true);
    if(txFailures >= MAX_TX_FAILURES){
      debug_print(F("ERROR"), F("Maximum transmition failures reached"), true);
      init_fona();
    }else{
      debug_print(F("STATUS"), F("Not reached"), true);
    }//end if
    previousMillis_2 = timer;
  }//end if
  //interrupts();//<-------?????27/10/2016
}//end check_connection

/*
 * ON and OFF functions
 * 
 * Tie  pin FONA_KEY to ground for 2 seconds to turn the module on or off.
 * It's not a level signal so it isn't like "low is off, high is on" 
 * instead you must pulse it for 2 seconds to turn off/on
*/

uint8_t fona_on(){
  while(digitalRead(FONA_PS)==LOW){//If FONA_PS still LOW, FONA is OFF
    digitalWrite(FONA_KEY, !digitalRead(FONA_KEY));
    delay(3000);
  }//end while
  digitalWrite(FONA_KEY, HIGH);
  return EXIT_SUCCESS;
}//end fona_on

uint8_t fona_off(){
  while(digitalRead(FONA_PS)==HIGH){//If FONA_PS is HIGH the FONA still ON
    digitalWrite(FONA_KEY, !digitalRead(FONA_KEY));
    delay(3000);
  }//end while
  digitalWrite(FONA_KEY, HIGH);
  return EXIT_SUCCESS;
}//end fona_off

uint8_t fona_network_status(){
  uint8_t network_status = fona.getNetworkStatus();
  if(network_status == 0){
    debug_print(F("WARNING"), F("Not registered"), true);
    return EXIT_FAILURE;    
  }else if(network_status == 1){
    debug_print(F("STATUS"), F("Registered (home)"), true);
    return EXIT_SUCCESS;
  }else if(network_status == 2){
    debug_print(F("WARNING"), F("Not registered (searching)"), true);
    return EXIT_FAILURE;    
  }else if(network_status == 3){
    debug_print(F("ERROR"), F("Denied"), true);
    return EXIT_FAILURE;        
  }else if(network_status == 4){
    debug_print(F("WARNING"), F("Unknown"), true);
    return EXIT_FAILURE;        
  }else if(network_status == 5){
    debug_print(F("WARNING"), F("Registered roaming"), true);
    return EXIT_FAILURE;        
  }//end if
}//end fona_network_status

uint8_t gprs_enable(void){
  /*
   * Turn GPRS ON, try 200 times until GPRS is ON, if that does not work
   * restart all system
  */
  for(uint8_t i = 0; i < 200; i++){
    if(fona.enableGPRS(true)){
      /* ==>TODO<==
       * Add to LOG # of attempts to achieve turn ON GPRS
      */      
      return EXIT_SUCCESS; //Indicates exit witout problems
    }//end if
  }//end for
  return EXIT_FAILURE;
}//end gprs_enable

uint8_t gprs_disable(){
  // turn GPRS off
  if (!fona.enableGPRS(false)){
    return EXIT_FAILURE;
  }//end if
  return EXIT_SUCCESS;
}//end gprs_disable

uint8_t fona_gps_on(void){                    //turn GPS on
  if (!fona.enableGPS(true)){
    return EXIT_FAILURE;//Failed to turn GPS on
  }//end if
  return EXIT_SUCCESS;
}//end fona_gps_on

uint8_t fona_gps_off(void){                    // turn GPS off
  if (!fona.enableGPS(false)){
    return EXIT_FAILURE;//Failed to turn GPS off
  }//end if
  return EXIT_SUCCESS;
}//end fona_gps_off

uint8_t fona_gps_location(){
  // check for GPS location, Latitude & longitude for distance measurement
  uint8_t gps_fix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  return gps_fix;
}//end fona_gps_location

uint8_t fona_sync_ntp(){// enable NTP time sync
  //NTP site : http://www.pool.ntp.org/en/use.html
  if (!fona.enableNTPTimeSync(true, F("pool.ntp.org"))){
    return EXIT_FAILURE;
  }//end if
  return EXIT_SUCCESS;
}//end if

uint16_t fona_get_battery(void){
  // Grab battery reading
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  return vbat;
}//end fona_get_battery

//======================Miscelanius Funcions==========================

void print_IMEI(void){
  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    debug_print(F("SIM card IMEI: "), F(imei));
  }//end if  
}//end print_IMEI

void update_initial_state(uint32_t &timer, uint32_t interval){
  /*
   * The worst scenario I see is 21s sending all messages
  */
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
    if(send_url(GPSlocation)){
      debug_print(F("WARNING"), F("Problem sending GPS location data"), true);
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

  if(send_url(BME280Data)){
    debug_print(F("WARNING"), F("Problem sending BME280 data"), true);
    txFailures++;
  }else{
    txFailures = 0;  
  }//end if    
  
  calcWeather();
  char weatherConditions[300];
  sprintf(weatherConditions, "&wind=%s&windSpeed=%s&rainin=%s&dailyrainin=%s", String(winddir_avg2m).c_str(), float_to_string(windspdmph_avg2m, 2).c_str(), float_to_string(rainin, 2).c_str(), float_to_string(dailyrainin, 2).c_str());

  if(send_url(weatherConditions)){
    debug_print(F("WARNING"), F("Problem sending Weather Conditions"), true);
    txFailures++;
  }else{
    txFailures = 0;  
  }//end if    
  
  String testMessage = "&battery="+String(fona_get_battery()) + lightSensorRead();
  if(send_url(testMessage.c_str())){
    debug_print(F("WARNING"), F("Problem sending Battery State"), true);
    txFailures++;
  }else{
    txFailures = 0;  
  }//end if    
  //------------------------------
    previousMillis_1 = timer;
  }//end if
  //interrupts();//<-------?????27/10/2016
}//end temporizer

uint8_t fona_gps_fix(){
  /*
   * Time to first fix 30s aprox.
  */
  for(uint8_t i = 0; i<200; i++){
    fona_gps_location();
    if(fona.GPSstatus() >= 2){
      return EXIT_SUCCESS;
    }//end if
    delay(10);
  }//end if
  return EXIT_FAILURE;
}//end fona_gps_fix

String lightSensorRead(){
  String light_sensor_data = "";
  if(uvInitialization){
    uint16_t temporal_save = light_sensor.readVisible();
    if(isnan(temporal_save)){
      debug_print(F("ERROR"), F("Reading Visible light [NAN]"), true);
    }else{
      //Read SI1145 UV Light
      light_sensor_data += "&visible=";
      light_sensor_data += temporal_save;
    }//end if

    temporal_save = light_sensor.readIR();
    if(isnan(temporal_save)){
      debug_print(F("ERROR"), F("Reading InfraRed light [NAN]"), true);
    }else{
      //Read SI1145 InfraRed Light
      light_sensor_data += "&ir=";
      light_sensor_data += temporal_save;
    }//end if
    
    temporal_save = light_sensor.readUV();
    if(isnan(temporal_save)){
      debug_print(F("ERROR"), F("error reading InfraRed light [NAN]"), true);
    }else{
      //Read SI1145 UV Light
      light_sensor_data += "&uv=";
      /*
       * the index is multiplied by 100 so to get the
       * integer index, divide by 100!
      */
      float UVindex = temporal_save;
      UVindex /= 100.0; 
      
      light_sensor_data += float_to_string(UVindex, 2);
    }//end if 
  }//end if
  return light_sensor_data;
}//end lightSensorRead

boolean BME280Initialization(){
  //**************Driver settings********************//
  
  weatherSensor.settings.commInterface = I2C_MODE;  //commInterface can be I2C_MODE or SPI_MODE
  
  weatherSensor.settings.I2CAddress = 0x77; //specify I2C address.  Can be 0x77(default) or 0x76  
  //*************Operation settings******************//
  /*
   * 0 Sleep Mode
   * 1 or 2 Forced mode
   * 3 Normal Mode
  */
  weatherSensor.settings.runMode = 3; //  3, Normal mode

  /*
   * tStandby can be:
   * 0.5 ms
   * 62.5ms
   * 125ms
   * 250ms
   * 500ms
   * 1000ms
   * 10ms
   * 20ms
  */
  weatherSensor.settings.tStandby = 0; //  0, 0.5ms

  /*Filter can be off or number of FIR coefficients to use
   * 0 Filter OFF
   * 1 Coef. =2
   * 2 Coef. = 4
   * 3 Coef. = 8
   * 4 Coef. = 16
   * 
   * Read about FIR coefficients http://dspguru.com/dsp/faqs/fir/basics
  */
  weatherSensor.settings.filter = 0; //  0, filter off
  /*
   * OverSample can be:
   * 0 skipped
   * 1 through 5, oversampling *1, *2, *4, *8, *16 respectively  
  */
  weatherSensor.settings.tempOverSample = 1;
  weatherSensor.settings.pressOverSample = 1;
  weatherSensor.settings.humidOverSample = 1;

  delay(10);//Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
   
  serial.println(weatherSensor.begin());//Calling .begin() causes the settings to be loaded
  //---------
  return true;
}//end BME280Initialization

void BME280Read(){
  /*Coud be better send all of this as a parameter?
   * BME280Read(float &bmpTemperature, float &bmpPressure, float &bmpAltitude, float &bmpHumidity)
  */
  bmpTemperature = weatherSensor.readTempC();
  if(isnan(bmpTemperature)){
    debug_print(F("ERROR"), F("Reading Temperature [NAN]"), true);
  }//end if
  bmpPressure =weatherSensor.readFloatPressure();
  if(isnan(bmpPressure)){
    debug_print(F("ERROR"), F("Reading Pressure [NAN]"), true);
  }//end if  
  bmpAltitude = weatherSensor.readFloatAltitudeMeters();
  if(isnan(bmpAltitude)){
    debug_print(F("ERROR"), F("Reading Altitude [NAN]"), true);
  }//end if
  bmpHumidity = weatherSensor.readFloatHumidity();
  if(isnan(bmpHumidity)){
    debug_print(F("ERROR"), F("error reading Humidity [NAN]"), true);
  }//end if
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
    //if(currentSpeed > windgust_10m[minutes_10m]){
    //  windgust_10m[minutes_10m] = currentSpeed;
    //  windgustdirection_10m[minutes_10m] = currentDirection;
    //}

    //Check to see if this is a gust for the day
    if(currentSpeed > windgustmph){
      windgustmph = currentSpeed;
      windgustdir = currentDirection;
    }
    if(++seconds > 59){
      seconds = 0;

      if(++minutes > 59) minutes = 0;
      //if(++minutes_10m > 9) minutes_10m = 0;
    }//end if
  }
}//end windIterate

float windSpeed(){
  //Returns the instataneous wind speed
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

uint8_t *fona_ntp_time_sync(const char *fona_time){
  //Split time format from FONA for configure in M0 RTC
  uint8_t date_numbers[6];//year, month, day, hour, minute, second
  uint8_t i = 1;
  for(uint8_t j= 0; j < 6; j++){
    char number[2];
    for(uint8_t k = 0; k < 3; k++){
      if(k == 2){//skip third caracter
        i++;
        break;
      }//end if
      number[k] = fona_time[i];
      i++;
    }//end for
    date_numbers[j] = atoi(number);//Convert chars to unsigned int
  }//end for
  return date_numbers;//Return array with date numbers
}//end fona_ntp_time

uint8_t rtc_m0_config(const char *fona_time){
  uint8_t calendar_time[6];//year, month, day, hour, minute, second
  memcpy(calendar_time,  fona_ntp_time_sync(fona_time), 6);//Move array of number 
  rtc.begin(true); // initialize RTC, always configure
  rtc.setDate(calendar_time[2], calendar_time[1], calendar_time[0]);//Set Date day, month, year
  rtc.setTime(calendar_time[3], calendar_time[4], calendar_time[5]);//Set Time hours, minutes, seconds
  return EXIT_SUCCESS;
}//end rtc_m0_config

void flushSerial() {
  #if defined(DEBUG)
  while (serial.available()) 
    serial.read();
  #endif
}//end flushSerial

String float_to_string(float value, uint8_t places) {//Adafruit funtion
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  //int i;
  float tempfloat = value;
  String float_obj = "";

  /*
   * Make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import 
   * if this rounding step isn't here, the value  54.321 prints as 54.3209
  */

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0){
    d *= -1.0;
  }
  
  for (uint8_t i = 0; i < places; i++){       // divide by ten for each decimal place
    d/= 10.0;
  }
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  /*First get value tens to be the large power of ten less than value 
   * tenscount isn't necessary but it would be useful if you wanted to know 
   * after this how many chars the number will take
  */
  
  if (value < 0){
    tempfloat *= -1.0;
  }
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }
  
  if (value < 0){                             //Write out the negative if needed
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
  
  if (places <= 0){                          //If no places after decimal, stop now and return
    return float_obj;
  }//end if
  
  float_obj += ".";                          //Otherwise, write the point and continue on

  /*
   * Now write out each decimal place by shifting digits one by one into 
   * the ones place and writing the truncated value
   */
  for (uint8_t i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    float_obj += String(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }//end for
  return float_obj;
}//end float_to_string

void restart(const __FlashStringHelper *error) {
  debug_print(F("RESETING"), F(error), true);                //Print message error 
  delay(1000);                          //Wait 1s for print pending message
  Watchdog.enable(1000);
  Watchdog.reset();
  while (1) {}                          //Wait "forever" until watchdog restart M0
}//end restart

//==========================Web Functions=============================
// Post data to website
uint8_t send_url(const char *raw_paq){
  /*For Initial State Event API please visite
   * http://docs.initialstateeventsapi.apiary.io/#
  */
  String url = "GET ";
  url += "/api/events?accessKey="+String(ACCESS_KEY)+"&bucketKey="+String(BUCKET_KEY);

  char c_url[800];
  sprintf(c_url, "%s%s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", url.c_str(), raw_paq, SERVER);

  if (fona.TCPconnect(SERVER, 80)) {         // if there's a successful connection:
    debug_print(F("STATUS"), F("Connecting with InitialState"), true);
    
    if(fona.TCPconnected()){                 // Make a HTTP request:
        debug_print(F("STATUS"), F("Sending"), true);
        debug_print(F("SENDING"), F(c_url), true);
        fona.TCPsend(c_url, strlen(c_url));
    }else {
      return EXIT_FAILURE;
    }//end if
  }
  delay(50);                                 //Wait a little for website response
  
  uint8_t max_buffer = 255;
  uint8_t response[max_buffer];              //The response could not be greather than 255 because TCPread() can't parse more than that
  uint16_t avail;

  while(avail = fona.TCPavailable()){
    if(!fona.TCPread(response, max_buffer)){
      break;//There is no message
    }//end if
  }//end while

  /*Print Response message
   * A 204 response 
  */
  debug_print(F("STATUS"), F("Initial State Response :"), true);
//  for(uint8_t i = 0; i< max_buffer; i++){
//    if(response[i] == '\0'){
//      break;
//    }//end if
  debug_print(F("ANSWER"), F(response), true);
//  }//end for
//  debug_print();
  
  fona.TCPclose();                           //Close TCP connection with InitialState
  return EXIT_SUCCESS;
}//end send_url

void debug_print(const __FlashStringHelper *status, const __FlashStringHelper *log_message, boolean send_to_openlog){
  char status_message[280];//Maximum char spected is 252, but include a little more
  /*
   * Set correct format for all message, if they go to openlog or not
   * Include RTC time in all messages
  */
  sprintf(status_message, "[%s] [%d/%d/%d - %d:%d:%d] : %s", status, rtc.getDay(), rtc.getMonth(), \
  rtc.getYear(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), log_message);
  serial.println(F(status_message));
  if(send_to_openlog){
    openlog.println(F(status_message));
  }//end if
}//end debug_print

//==========================Interrupt routines========================
void rainIRQ(){
  // Count rain gauge bucket tips as they occur
  // Activated by the magnet and reed switch in the rain gauge

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


