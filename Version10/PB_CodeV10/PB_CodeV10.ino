
// select board AdaFruit ItsyBitsy MO
#include <Adafruit_SleepyDog.h>
// Data recording
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);
// file system object from SdFat
FatVolume fatfs;
// Configuration for the datalogging file:
#define FILE_NAME      "data.csv"

// GPS
#include <TinyGPS++.h>
//#include <RTClib.h>
//#include <Timezone.h>
float lat,lon,sped;
int qual;
unsigned long age, date, Time;
static void print_date(TinyGPSPlus &gps);
TinyGPSPlus gps;// create gps object
// Change these two rules corresponding to your timezone, see https://github.com/JChristensen/Timezone
//Australian Western Standard Time (Perth)  +8
//TimeChangeRule AWST = {"AWST", First, Sun, Mar, 2, 0};
//Timezone CE(AWST, AWST);
// RTC
#include "GravityRtc.h"
#include "Wire.h"
GravityRtc rtc;              //RTC Initialization
// Control the NeoPixel
#include <Adafruit_DotStar.h>
#define NUMPIXELS 1   // There is only one pixel on the board
#define DATAPIN    41
#define CLOCKPIN   40
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

// Temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS A1
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// Pins
int curr_read =    0;     // was 1 read if Current flows A1
int temp_read =    A1;    // new pin for temperature
int curr_out =     11;    //  current send
int curr_quite =   12;    //  stay low - provide reference current
int camera =   7;         //  was ??
int GPS =    5;         //  was 9. 
int LED =   13;           //  
int Current   =    0;    //  Record the current in
int pwrlsensor =   9;    // powerup the light sensor
int lsensor =     A3;    //  lightsensor

// Print , Clear, Run
String command = "Run";
int start_potbot = -1;        // Used to designate whether initial deployment has occurred or if the unit should wait. 
int vidtaken = 0;                    //  Used to determine whether a video has been taken in that loop.
int temptaken = 0;                   //  Used to determine whether a temperature recording has been taken in that hour.
unsigned long TimeDeployed = 0;     //    Record when the temperature was taken for a lag
unsigned long Time4Temperature = 0;     //    Record when the temperature was taken for a lag
int Deployed = 0;            //  Used to determine if just deployed
int DeployedNo = 0;  //  Number of times POTBot has been deployed
int Num = 0;         
bool BoOL = true;
int Retrieved = 0;            //  Used to determine if just retrieved
int FstGPSLock = 0;            //  Used to determine if just got a lock
//bool inwater();                      // Function to determine whether the POTBot is in the water.
//bool VidNowEarly();                    // Function to determine whether the deployment video was taken in the dark.
bool Vid_early = false;             // To record whether the video was taken in the dark
//bool CommandRun();
String Text = "Hello";
float Lat = 0; float AvLat = 0;     // Average latitude of 5 positions
float Lon = 0; float AvLon = 0;     // Average longitude of 5 positions
unsigned long GPSstart = 0;
int sensorValue = 0;                // For voltage tracking
float voltage = 0;
float voltagevar = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long VidTime = 2400;      // Seconds
long GPSTime = 240;      // Minutes
int Dawn = 7;
int Dusk = 17;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
   // Neo Pixel
   strip.begin(); // Initialize pins for output
   strip.setBrightness(0);
   strip.setPixelColor(0, strip.gamma32(strip.ColorHSV(28160)));
   strip.show();  // Turn all LEDs off ASAP
  
   pinMode(curr_read, INPUT);
   pinMode(curr_out, OUTPUT);
   pinMode(curr_quite, OUTPUT);
   pinMode(camera, OUTPUT);
   pinMode(GPS, OUTPUT);
   pinMode(LED, OUTPUT);
   pinMode(temp_read, INPUT);
   pinMode(lsensor, INPUT);
   pinMode(pwrlsensor, OUTPUT);
   
   rtc.setup();
   //Set the RTC time automatically: Calibrate RTC time by your computer time
   //rtc.adjustRtc(F(__DATE__), F(__TIME__));
   
  Serial.begin(9600);
  sensors.begin();
  delay(1000);
  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1) delay(1);
  }
//  Serial.print("Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
    while(1) delay(1);
  }
 // Serial.println("Mounted filesystem!");
 //  Serial.println("Logging data every 60 seconds...");

 // Serial.begin(57600); // connect serial
 // Serial.println("The GPS Received Signal:");
  Serial1.begin(9600); // connect gps sensor
  // Check voltage
  sensorValue = analogRead(A2);
  voltage = sensorValue * (3.3 / 1023.0) * 2.0; // voltage halved and then compared to 3.3 
  if(voltage>4.0) {
    strip.begin(); // Initialize pins for output
    strip.setBrightness(100);
    strip.setPixelColor(0, strip.gamma32(strip.ColorHSV(0)));  // Green
    strip.show();
  } else {
    strip.begin(); // Initialize pins for output
    strip.setBrightness(100);
    strip.setPixelColor(0, strip.gamma32(strip.ColorHSV(20480)));  // Red
    strip.show();
  }//9984
  delay(5000);
  Serial.println("The POTBot is loaded with code V10.");
  Serial.print("It will film for ");Serial.print(VidTime/60);Serial.println(" minutes when deployed.");
  PrintDate("The RTC Date/Time is: ");
  Serial.println("To stop the POTBot and print out stored data enter 'Print'.");
  Serial.println("To stop the POTBot clear data enter 'Clear'.");
  //Serial.println("To stop the POTBot clear data enter 'Clear'.");
  strip.setBrightness(0);
  strip.show();
  if(!Serial) {PrintVoltageDF();}
}

void loop() {
    if(!Serial){ Watchdog.sleep(1000);}   // Run sleep if the serial cable is not connected
////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
   // Reads input entered through the Serial Monitor     
   if(Serial.available())  { 
      command = Serial.readStringUntil('\n'); 
      command.trim();}
 // This prints the data storage if "Print" in entered through the serial Monitor     
   if(command.equals("Print")){
      File32 dataFile = fatfs.open(FILE_NAME, FILE_READ);
      if (dataFile) {
      Serial.println(" "); Serial.println("Opened file, printing contents below:");Serial.println(" "); 
      while (dataFile.available()) {
       char c = dataFile.read();
       Serial.print(c);
    } }  else {
    Serial.print("Failed to open file \"");    Serial.print(FILE_NAME);   Serial.println("\" !! Does it exist?");
  }
     delay(1000);
     command = "Finished";
      }  
 // This clear the data storage if "Clear" in entered through the serial Monitor     
   if(command.equals("Clear")){
     Serial.println(" "); Serial.println("Deleting data file");Serial.println(" "); 
     if(!fatfs.remove("data.csv")) {
        Serial.println("Error, couldn't delete test.txt file!");
        while(1) yield();
       }
    delay(1000);
    command = "Finished"; 
      }  

// Initial GPS search to try and get a lock and improve subsequent gps efficiency.
while(start_potbot == -1 && CommandRun()) {  
   digitalWrite(GPS, HIGH);  // Start the GPS
   // Serial.println("Searching for satellites to initiate the GPS ");
   // Set LED to yellow whilst search for GPS
    strip.setBrightness(100);
    strip.setPixelColor(0, strip.gamma32(strip.ColorHSV(10922))); // Yellow when searching
    strip.show();
   if(Serial1.available()){                // check for serial connection - feed from GPS
    if(gps.encode(Serial1.read())) {     // encode gps data
      if(gps.location.isUpdated() && gps.satellites.value() >= 3) {
        lat = gps.location.lat();
        lon = gps.location.lng(); 
        sped = gps.speed.mps();
        qual = gps.satellites.value();
        strip.setPixelColor(0, strip.gamma32(strip.ColorHSV(54613))); // Purple
        strip.show();
        delay(60000);                // Wait one minute with the led on to highight a connection has been made
        rtc.read();

     // Get current rtc time
        if(gpsFixIsReliable() && !(rtc.minute==gps.time.minute()) && gps.time.second()<50 && rtc.second<50) { resetRTC(); }

  digitalWrite(GPS, LOW);  // Turn off the GPS
  Serial.println("Satellites found and GPS initiated. Ready to be deployed.");
  // Turn LED off
  strip.setBrightness(0);
  strip.show();
  start_potbot = 0; // Let the  fun begin
        }}}
}


      
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Test is wet
      while(inwater() && CommandRun()) {                  // Camera is wet and POTBot set to "Run"
        rtc.read();
        start_potbot = 1;                                 // Once deployed start POTBot on normal cycle so it will enter the GPS once retrieved.
        FstGPSLock=0;
        if(Deployed==0) {                                 //  Has just been deployed and not yet taken video
          DeployedNo += 1;
          PrintDate("I am in the water.");
          Time4Temperature = rtc.hour + 2.0;  // Set time to record temperature 2 hours after deployment
          if (Time4Temperature>23) { Time4Temperature = Time4Temperature-24;} 
          PrintDateDF("POTBot has been deployed. ", true);
          PrintTextNum("Deployment number: ",DeployedNo);
          if(DeployedNo>1){ PrintPosDF("Last GPS position recorded before deployment (Lat, Lon, Speed, NSat.): ", lat, lon, sped, qual);}                        // Record last position before deployment                                        
          }    
        Deployed=1; Retrieved=0;
        // Take video following deployment
        if(vidtaken==0){                                     // Has a video yet been taken in this deployment? This is reset in the GPS code
          Vid_early = VidNowEarly(Dawn,Dusk);                     // Logical, is the video being taken outside of Dawn and Dusk
          digitalWrite(camera, HIGH);
          delay(VidTime*1000);
          digitalWrite(camera, LOW);
          vidtaken=1;
          PrintDateDF("Video Taken.", false);
          PrintVoltageDF();
          }
        // Take second video at 9 am if first video was taken in the dark
         if(Vid_early && !VidNowEarly(11,Dusk)){                 // If the video was taken outside of Dawn and Dusk, then take another video once it gets light - somewhere in the start, end times listed in the brackets
          digitalWrite(camera, HIGH);
          delay(120*1000);                                  // 120 seconds just to get the habitat
          digitalWrite(camera, LOW);
          Vid_early = false;
          PrintDate("Short Video Taken because deployment video was taken in the dark.");
          PrintDateDF("Short Video Taken because deployment video was taken in the dark.", false);
           }
         if(Time4Temperature==rtc.hour  &&  rtc.minute==0 && rtc.second<5){             // Take first temperature 90 minutes after deployment
          PrintTempDF();
          PrintDateDF(" Water temperature", false);
          delay(100);
          Time4Temperature = 23; // Just record at 11 pm every night thereafter   rtc.hour + 6.0;  if (Time4Temperature>24) { Time4Temperature = Time4Temperature-24;}
            } 
         if(!Serial){ Watchdog.sleep(1000);}   // Run sleep if the serial cable is not connected
        }            // In the water
        
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        
// Is not wet, has been deployed?    
  while(!inwater() && start_potbot == 1 && CommandRun()) {                     // Not in the water     
  //Serial.println("I am high and dry");
    rtc.read();                             
    if(Retrieved==0) {
        PrintDateDF("POTBot has been retrieved and GPS Started. ", false);      // Record the retrieval time
        GPSstart = millis();
        }                                                   // Record when the GPS started
      if((millis() - GPSstart) > (GPSTime * 60000)){                               // If GPS times out, reset POTBot and record the shutdown
         start_potbot = 0;
         PrintDateDF("POTBot GPS has timed out. ", false); }   
      Retrieved=1; Deployed=0; vidtaken=0;                                     // Set back to 0 so next time it records when it went into the water// Reset video taken
      digitalWrite(GPS, HIGH);                                                 // Start GPS
      
    // while(!Serial1.available())  { Serial.println("No GPS signal yet.");  }
    if(Serial1.available()){                // check for serial connection - feed from GPS
      if(gps.encode(Serial1.read())) {     // encode gps data
        if(gps.location.isUpdated() && gps.satellites.value() >= 3) {
          lat = gps.location.lat();
          lon = gps.location.lng(); 
          sped = gps.speed.mps();
          qual = gps.satellites.value();
          //Serial.println("Got position will save to file.");
          if(FstGPSLock<=2){                           // Record first five updates
            if(FstGPSLock==0) { PrintDateDF("GPS found a lock. Printing first three locations.", false); }
            PrintPosDF("Lat, Lon, Speed, NSat.: ",lat, lon, sped, qual);
            FstGPSLock+=1;                                  // Set to 1 to stop further printing of GPS
            }
         
        delay(500);
        rtc.read();
     // Get current rtc time
        if(gpsFixIsReliable() && !(rtc.minute==gps.time.minute()) && gps.time.second()<50 && rtc.second<50) { resetRTC(); }
        }}
      } 
     //delay(500);
    }  // End of not wet
   
    digitalWrite(GPS, LOW);                            // Turn off GPS and LED
     }     // void loop

bool inwater() {
    bool tmpbool = false; 
    digitalWrite(curr_out, HIGH); 
    Current = analogRead(curr_read);
    digitalWrite(curr_out, LOW); 
    if(Current>20) { 
      if(!Serial){ Watchdog.sleep(2000);}             // Run sleep if the serial cable is not connected
        digitalWrite(curr_out, HIGH); 
        Current = analogRead(curr_read);
        digitalWrite(curr_out, LOW);
        if(Current>20) { if(!Serial){ Watchdog.sleep(2000);}             // Run sleep if the serial cable is not connected
          digitalWrite(curr_out, HIGH); 
          Current = analogRead(curr_read);
          digitalWrite(curr_out, LOW);
          if(Current>20) { if(!Serial){ Watchdog.sleep(2000);}             // Run sleep if the serial cable is not connected
            digitalWrite(curr_out, HIGH); 
            Current = analogRead(curr_read);
            digitalWrite(curr_out, LOW);
            if(Current>20) { tmpbool = true; } } }
      } 
    return tmpbool; 
       }

bool VidNowEarly(int dawn, int dusk) {
      rtc.read();
      if (rtc.hour>=dawn && rtc.hour<=dusk) return false ;
    else  return true; }

bool CommandRun(){
   if(Serial.available())  { 
      command = Serial.readStringUntil('\n'); 
      command.trim();}
      if(command.equals("Run")) return true;
      else  return false;}


bool gpsFixIsReliable() {
  if (!gps.date.isValid() || !gps.time.isValid()) return false;  // GPS hasn't got a proper lock
  if (qual < 4) return false;                                      // Fewer than 4 satellites — unreliable
  // Sanity check: GPS year should be within 1 year of RTC year (catches stale cached fixes)
  rtc.read();
  int gpsYear = gps.date.year();
  int rtcYear = rtc.year;
  if (abs(gpsYear - rtcYear) > 1) return false;                   // Date is wildly wrong
  return true;
}


void PrintDate(String Text){
  rtc.read();
  Serial.print(Text);
  Serial.print(rtc.year);Serial.print("-");Serial.print(rtc.month); Serial.print("-");Serial.print(rtc.day);Serial.print("  "); Serial.print(rtc.hour); Serial.print(":");Serial.print(rtc.minute);Serial.print(":");Serial.println(rtc.second);
  }

void PrintDateDF(String Text, bool BoOL){
  rtc.read();
  File32 dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  if (dataFile) {  
    if(BoOL) {dataFile.println(" ");}
    dataFile.print(Text);dataFile.print(" ");
    dataFile.print(rtc.year);dataFile.print("-");dataFile.print(rtc.month); dataFile.print("-");dataFile.print(rtc.day);dataFile.print("  "); dataFile.print(rtc.hour); dataFile.print(":");dataFile.print(rtc.minute);dataFile.print(":");dataFile.println(rtc.second);}
  dataFile.close();
  }

void PrintPosDF(String Text, float AvLat, float AvLon, float speed, float quality){
  File32 dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  if (dataFile) { dataFile.print(Text);dataFile.print(" ");dataFile.print(AvLat,7);dataFile.print(", ");dataFile.print(AvLon,7);dataFile.print(", ");dataFile.print(speed,5);dataFile.print(", ");dataFile.println(quality,5); }
  dataFile.close();
  }

void PrintVoltageDF(){
  sensorValue = analogRead(A2);
  voltage = sensorValue * (3.3 / 1023.0) * 2.0; // voltage halved and then compared to 3.3
  File32 dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  if (dataFile) { dataFile.print("Voltage: ");dataFile.print(voltage);dataFile.println(" v"); }
  delay(300);
  dataFile.close();
  }

void PrintTempDF(){
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures(); 
  File32 dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  if (dataFile) { dataFile.print(sensors.getTempCByIndex(0)); }
  dataFile.close();
  }

void PrintTextNum(String Text, int Num){
  File32 dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  if (dataFile) { dataFile.print(Text);dataFile.println(Num);}
  dataFile.close();
  }

void resetRTC() {
  int  hours;
  int  minut,seconds,yer,mnth,dte,week;
  yer = 0;
  hours = 0;
  hours=gps.time.hour();
  minut= gps.time.minute();
  seconds=gps.time.second();  // }
  mnth=gps.date.month();
  yer=gps.date.year();
  dte=gps.date.day();
  week=calcDayOfWeek((yer-2000), mnth, dte); // }
  if(yer!=0 && hours!=0) {                      //UTC time is converted GMT+5.5(Indian time)
    int mon[]={31,28,31,30,31,30,31,31,30,31,30,31};
    if (((yer % 4) == 0) && ( ((yer % 100) != 0) || ((yer % 400) == 0) ))  {  mon[1]=29; } else { mon[1]=28;}
    hours=hours+8;
    if(hours>=24) {
      hours=hours-24;
      dte=dte+1; }
    if(dte>mon[mnth-1]) {
        dte=dte-mon[mnth-1];
        mnth=mnth+1; }
    if(mnth>12) {
      mnth=1;
      yer=yer+1; }
    
    File32 dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
    if (dataFile) {  
      dataFile.println("Updating the RTC from the GPS"); 
      dataFile.print("RTC time:");dataFile.print(rtc.year);dataFile.print("-");dataFile.print(rtc.month); dataFile.print("-");dataFile.print(rtc.day);dataFile.print("  "); dataFile.print(rtc.hour); dataFile.print(":");dataFile.print(rtc.minute);dataFile.print(":");dataFile.print(rtc.second);
      dataFile.print(" GPS time:");dataFile.print(yer);dataFile.print("-");dataFile.print(mnth); dataFile.print("-");dataFile.print(dte);dataFile.print("  "); dataFile.print(hours); dataFile.print(":");dataFile.print(minut);dataFile.print(":");dataFile.println(seconds);
      }
    dataFile.close();
    rtc.adjustRtc(yer, mnth, dte, week, hours,minut,seconds);
      }     }

byte calcDayOfWeek (byte y, byte m, byte d) {
  // Old mental arithmetic method for calculating day of week
  // adapted for Arduino, for years 2000~2099
  // returns 1 for Sunday, 2 for Monday, etc., up to 7 for Saturday
  // for "bad" dates (like Feb. 30), it returns 0
  // Note: input year (y) should be a number from 0~99
  if (y > 99) return 0; // we don't accept years after 2099
  // we take care of bad months later
  if (d < 1) return 0; // because there is no day 0
  byte w = 6; // this is a magic number (y2k fix for this method)
  // one ordinary year is 52 weeks + 1 day left over
  // a leap year has one more day than that
  w += (y + (y >> 2));                    // we add in these "leftover" days
  if (((y & 3) == 0) && (m <= 2)) w--;   // correction for Jan. and Feb. of leap year
  // add in "magic number" for month
  switch (m) {
    case 1:  if (d > 31) return 0; w += 1; break;
    case 2:  if (d > ((y & 3) ? 28 : 29)) return 0; w += 4; break;
    case 3:  if (d > 31) return 0; w += 4; break;
    case 4:  if (d > 30) return 0; break;
    case 5:  if (d > 31) return 0; w += 2; break;
    case 6:  if (d > 30) return 0; w += 5; break;
    case 7:  if (d > 31) return 0; break;
    case 8:  if (d > 31) return 0; w += 3; break;
    case 9:  if (d > 30) return 0; w += 6; break;
    case 10: if (d > 31) return 0; w += 1; break;
    case 11: if (d > 30) return 0; w += 4; break;
    case 12: if (d > 31) return 0; w += 6; break;
    default: return 0;
  }
  // then add day of month
  w += d;
  // there are only 7 days in a week, so we "cast out" sevens
  while (w > 7) w = (w >> 3) + (w & 7);
  w = w -1;  //  Needs to be 0 - 6
  return w;
}




