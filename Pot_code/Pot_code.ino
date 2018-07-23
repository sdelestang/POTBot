#include "Time.h"
#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

//Board Potbotv6
#include <SoftwareSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include "TinyGPS.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_LSM303_U.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>


#define EEPROM_addrs  1
#define SW_TOL  3			//inverted logic
#define SW_GPS  4           //inverted logic
#define SW_CAM  5			//inverted logic
#define SW_MAG  7
#define LED 8				// LED state
// #define VI_ACC  A1
#define TX_GPS  13
#define RX_GPS  12
#define RX_OL  11
#define TX_OL  10
#define I_VBUS  A3
#define I_CH1  A2
// A2 Not used
#define I_HUM  2
#define temp_add 72
#define GPS_THRESHOLD 3
#define ACC_OFFSET 15000 // all positives values

SoftwareSerial gps_serial(RX_GPS, TX_GPS); // RX, TX   // GPS  12 13
SoftwareSerial ol_serial(RX_OL, TX_OL); // RX, TX   // OL
//time
TinyGPS gps;
byte ol_status=0;   //0 nothing    1 CMD mode    2 Append
bool screen=0;
bool logger=0;
char gps_thresh=0;
float latitude, longitude, temperature;                 // TIME IN SECONDS FOR NOW!!
             //0 dummy    ,1 Video_time [min], 2 temp_freq [min], 3 mag_value [], 4 mag_axis [0,1,2], 5 Video_time [min], 6 GPS_time out [min], 7 mag_value [], 8 mag_axis [0,1,2], 9 Pre_dep_T,   10 Pre_dep t
int parameters[11]={0,         5,                  5,                 60,              0,                  5,                  300,                  60,              0,               300,         300 };

float read_temp(int *val, int *frac);
void log_acc();
int get_mag(int axis, bool print_on, bool loop_on);
void get_acc(float acc_float[], bool print_on, bool loop_on);
int get_gps(float *latitude, float *longitude);
int command(char data);
void digitalClockDisplay();
void printDigits(int digits);
int ol_cmd();
int ol_append();
int ol_potbot_parameters(int num_parameters);
int get_acc_threshold(bool serial_print);
byte sequence(byte state);
void ol_timestamp();
void ol_printDigits(char separator, int digits);
void ol_logevent(int event);
void sleep_ex();
void sleep_WD();
bool mag_threshold(int value, int axis);
void LED_blink(int blinks, unsigned long delay_time);
bool serial_mode=0;
bool sequence_mode=0;
//Accelerometer config
/* Assign a unique ID to this sensor at the same time */

void setup()
{
  char data;
  
  pinMode(SW_TOL, OUTPUT);
  pinMode(SW_GPS, OUTPUT);
  digitalWrite(SW_GPS,HIGH);   //INITIALIZE GPS OFF
  digitalWrite(SW_GPS,HIGH);   //INITIALIZE GPS OFF
  digitalWrite(SW_GPS,HIGH);   //INITIALIZE GPS OFF
  pinMode(SW_CAM, OUTPUT);
  digitalWrite(SW_CAM,HIGH);   //INITIALIZE GPS OFF
  pinMode(I_HUM, INPUT);
  pinMode(I_VBUS, INPUT);
  pinMode(I_CH1, INPUT);
  pinMode(SW_MAG,OUTPUT);
  digitalWrite(SW_GPS,HIGH);   //INITIALIZE GPS OFF

  // initialize 
  Serial.begin(4800);
  if(digitalRead(I_VBUS)) 
  	command('c');

  Wire.begin();
  // initialize the serial communication:

  gps_serial.begin(4800); 
  ol_serial.begin(4800);
  delay (1000);

  if(Serial)
	Serial.println(F("PotBot v_7.1"));
  ol_potbot_parameters(10);
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  ol_logevent(0);
   //wait for command
  Serial.println(F("Wait for command: 5s"));
  for (unsigned long start = millis(); millis() - start < 5000;)
  {
  if (Serial.available()){
		command('C');
		data   = Serial.read();
		Serial.write(data);
		Serial.print("   :");
		command(data);
		serial_mode=1;
		break;
		}
  }
   Serial.println(F("Finish init"));
}

void loop() {
  static byte state=0;
  char data;
	
  if (screen)                        // OPENLONG SCREEN INTERFACE
	   {  if (ol_serial.available()) {
		Serial.write(ol_serial.read());
		}
		if (Serial.available()) {
		ol_serial.write(Serial.read());
		 }
	  }
  	else if (logger)   //LOGGER
	{
	  // log_acc();
	  get_mag(0, 1, 0);

	}   
	else if (serial_mode){
  	  if (Serial.available())
  	  {
		data   = Serial.read();
		Serial.write(data);
		Serial.print("   :");
		command(data);
	  }
	}
  	else if (!digitalRead(I_VBUS) || sequence_mode) {
		  //Normal 

		  state=sequence(state);
		  if ((state!=10)&& (state>2)) sleep_WD();  // sleep watchdog
		}
		return;
}

float read_temp(int *val, int *frac){
	  float temperature;
	  digitalWrite(SW_TOL, LOW);
	  delay(1000);
	  // Serial.print("\tTemp & Open log:  ");	//ZZZ delete
	  // Serial.print(digitalRead(SW_TOL),DEC);	//ZZZ delete
		// Serial.print("\n");										//ZZZ delete
	  // Serial.print("TEMP ON \n");  //ZZZ delete
	  delay(100);
	   // Serial.print("begin tx \n");  //ZZZ delete
	  Wire.beginTransmission(temp_add);
	  // send configuration
	  // Serial.print("send config \n");  //ZZZ delete
	  Wire.write(0xAC);
	  Wire.write(B00001111); // 12 bit resolution, pol, oneshot
	  Wire.endTransmission();
	  delay(10);
	  for(int i = 1 ; i < 5 ; i++) {   //read 3 times
	  
	  // begin convert
	  delay(50);
	   // Serial.print("begin convert \n");  //ZZZ delete
	  Wire.beginTransmission(temp_add); 
	  Wire.write(0x51);
	  Wire.endTransmission(); 
	  delay(20);
	 
	  // wait until converting is done
	   // Serial.print("Wait for convertion \n");  //ZZZ delete
	  byte conf = 0;
	  while ( conf & B1000000 != B10000000 ) {
		delay(20);
		Wire.beginTransmission(temp_add); 
		Wire.write(0xAC); 
		Wire.endTransmission();
		conf = Wire.read();
		}

	  // Serial.print("Query temperature \n");  //ZZZ delete
	  // ask for the temerature 
	  Wire.beginTransmission(temp_add); 
	  Wire.write(0xAA); 
	  Wire.endTransmission();

	  // Serial.print("Read bytes \n");  //ZZZ delete
	  // request 2 bytes
	  Wire.requestFrom(temp_add, 2);
	  // read first byte
	 
	  *val = Wire.read();
	  
	  // read second byte
	  *frac = Wire.read();
	  delay(100);
	  }
	  if (*frac>100)
		temperature=(float)*frac/1000;
	  else if (*frac>10)
		temperature=(float)*frac/100;
	  else
		temperature=(float)*frac/10;
	  temperature+=(float)*val;
	  Serial.print("\t  Temp: ");
	  Serial.println(temperature,3);
	  *frac = 100 * (*frac & 0xF0 )/ 256;
	  if ( *frac < 10 ) {

		}
	  delay(50);
	  digitalWrite(SW_TOL, HIGH);  //OFF
	delay(50);
  return temperature;
}

void log_acc(){
	Serial.println(F("\tNot used"));
	// 
	// Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
	// delay(50);
	// sensors_event_t event_acc;
	// accel.getEvent(&event_acc);
	// sensors_event_t event_mag;
	// mag.getEvent(&event_mag);
	// //Serial.print("logging... \n");  //ZZZ delete

	// ol_serial.print(event_acc.acceleration.x);
	// ol_serial.print(";");
	// ol_serial.print(event_acc.acceleration.y);
	// ol_serial.print(";");
	// ol_serial.print(event_acc.acceleration.z);
	// ol_serial.print(";");
	// ol_serial.print(event_mag.magnetic.x);
	// ol_serial.print(";");
	// ol_serial.print(event_mag.magnetic.y);
	// ol_serial.print(";");
	// ol_serial.print(event_mag.magnetic.z);
	// ol_serial.println("");
}

int get_mag(int axis, bool print_on, bool loop_on){
  static Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
  static bool init_acc_mag=0;
  mag.enableAutoRange(true);  
  int mag_int[3];
  if (!init_acc_mag)
  {
	mag.begin();
	delay(200);
	init_acc_mag=1;
  }

	sensors_event_t event_mag;
	while(1)
	{
	  delay(1000);
	  mag.getEvent(&event_mag);
	//Serial.print("logging... \n");  //ZZZ delete
	  if (print_on)
	  {
		Serial.print("\t\t");
		Serial.print(event_mag.magnetic.x);
		Serial.print(";");
		Serial.print(event_mag.magnetic.y);
		Serial.print(";");
		Serial.print(event_mag.magnetic.z);
		Serial.println(" uT");
	  }
	  if (!loop_on) break;
	}
	mag_int[0]=(int)(event_mag.magnetic.x*100.0);
	mag_int[1]=(int)(event_mag.magnetic.y*100.0);
	mag_int[2]=(int)(event_mag.magnetic.z*100.0);
		// Serial.print("\n\t\t");
		// Serial.print(mag_int[0]);
		// Serial.print(";");
		// Serial.print(mag_int[1]);
		// Serial.print(";");
		// Serial.print(mag_int[2]);
	return mag_int[axis];
}

void get_acc(float acc_int[], bool print_on, bool loop_on){

  static Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
  static bool init_acc_mag=0;
  

  if (!init_acc_mag)
  {
	accel.begin();
	init_acc_mag=1;
  }

	sensors_event_t event_acc;
	while(1)
	{
	  delay(200);
	  accel.getEvent(&event_acc);
	  

	  //Serial.print("logging... \n");  //ZZZ delete
	  if (print_on)
	  {
		Serial.print(event_acc.acceleration.x);
		Serial.print(";");
		Serial.print(event_acc.acceleration.y);
		Serial.print(";");
		Serial.print(event_acc.acceleration.z);
		Serial.print(" m/s^2;");
	  }
	  if (loop_on) break;
	}
	acc_int[0]=accel.raw.x;
	acc_int[1]=accel.raw.y;
	acc_int[2]=accel.raw.z;
	return;
}



/// get_gps:
//Read GPS for a second and 
//return 1: valid sentence recieved 
//return 0: no valid
// latitude and longitude as floats
int get_gps(float *latitude, float *longitude)
{
	gps_serial.listen();
	// Serial.println("GPS Start");
   bool newData = false;
   int year;
   byte month, day, hour, minutes, second, hundredths;
   unsigned long fix_age;
   
   // For one second we parse GPS data and report some key values
   for (unsigned long start = millis(); millis() - start < 2000;)
   {
	 while (gps_serial.available())
	 {
		 char c = gps_serial.read();
	  //delay(50);
	  // Serial.print(c); // uncomment this line if you want to see the GPS data flowing
	   if (gps.encode(c)) // Did a new valid sentence come in?
	   {
		  unsigned long age;
		  gps.f_get_position(latitude, longitude, &age);
		  gps.crack_datetime(&year, &month, &day,
		  &hour, &minutes, &second, &hundredths, &fix_age);
		  setTime(hour,minutes,second,day,month,year);
		  setTime(now()+28800);
		  Serial.println(F("\tGPS received"));
		  command('G');   //GPS OFF
		  return 1;
	   }
	 }  
   }
	// Serial.println("Timed out");
	return 0;
  
 }

int command(char data){
  switch (data){
	case 'a': 
	  // digitalWrite(SW_ACC, !digitalRead(SW_ACC));
	  // Serial.print(digitalRead(SW_ACC),DEC);
	  // Serial.print("  ACC\n");
	break;

	case 't': 
	  Serial.print(digitalRead(SW_TOL),DEC);
	  Serial.print("\tTemp & Open log\n");
	  int val, frac;
	  temperature=read_temp( &val, &frac);
	  
	  break;

	case 'T': 
	  digitalWrite(SW_TOL, HIGH);   //off
	  Serial.print(digitalRead(SW_TOL),DEC);
	  Serial.print(F("\tLOGGER OFF \n"));
	  break;

	case 's': 
	  sequence_mode=!sequence_mode;
	  break;

	case 'g': 
	  digitalWrite(SW_GPS, LOW);
	  Serial.println(F("\tGPS on..."));
	  gps_serial.listen();
	  delay(50);
  break;
  
  case 'G': 
	  digitalWrite(SW_GPS, HIGH);
	  Serial.println(F("\tGPS off..."));
	  delay(50);
  break;
	  
	case 'c':
	  digitalWrite(SW_CAM, LOW);
	  Serial.print(F("\tCamera on \n"));
	  delay(50);
  break;

  case 'C':
	   digitalWrite(SW_CAM, HIGH);      
	   Serial.println(F("\tCamera off"));
	   delay(50);
  break;
  
	case 'h':
	  Serial.print(digitalRead(I_HUM),DEC);
	  Serial.print(F("\thum\n"));
	  break;


	case 'w':
	  digitalWrite(SW_TOL, LOW);               //ON
	  Serial.print(F("\tOpen log writing test\n"));
	  delay(1000);
	  for(int i = 1 ; i < 10 ; i++) {
	  ol_serial.print(i, DEC);     
	  ol_serial.println(F(":abcdefsdff-!#"));
	  delay(100);
	  }
	  
	  delay(1000);
	  Serial.print(F("\tfinish writing\n"));
	  digitalWrite(SW_TOL, HIGH);   // OFF

	  break;

	 case 'o':                                   //screen
	 digitalWrite(SW_TOL, LOW);
	 ol_serial.listen();
	 ol_status=0;
	 ol_cmd();
	 screen=1;
	 
	 
	 
	 break;


	 case 'l':                                   //log
	 Serial.println(F("\tprint logger"));
	//  digitalWrite(SW_TOL, HIGH);
	//  digitalWrite(SW_ACC, LOW);
	//  delay(500);
	//   Serial.print("  Logger: ");
	//   Serial.print(!logger,DEC);
	//   Serial.println("");
	//   ol_serial.listen();
	
	// if (!logger){
	//    for (int i=0; i<20 ; i++){
	//      ol_serial.println("");
	//      ol_serial.write(26);
	//      ol_serial.write(26);
	//      ol_serial.write(26);
	//      ol_serial.println("");
	//      if (ol_serial.available())
	//     //Serial.println(ol_serial.read());
	//       if(ol_serial.read()=='>')
	//         {Serial.print(i, DEC); Serial.println(">"); break;}
	//      }
	//    delay(20);
	//    int value = EEPROM.read(EEPROM_addrs);
	//    EEPROM.write(EEPROM_addrs, value+1);
	//    ol_serial.println("");
	//    delay(10);
	//    ol_serial.println("");
	//    delay(10);
	//    ol_serial.print("append mylog_");
	//    if (value<10) ol_serial.print("0");
	//    ol_serial.print(value,DEC);
	//    ol_serial.println(".txt");
	//    delay(10);
	// }else
	// {
	//      ol_serial.write(26);
	//      ol_serial.println("");
	//      delay(10);
	// }
	//     ol_serial.println("x (m/s^2);y (m/s^2);z (m/s^2);x (uT);y (uT);z (uT)");
	logger=!logger;
	 break;

	 // case 'r':
	 //  EEPROM.write(EEPROM_addrs, 0);
	 //  Serial.print("  reset logger file to new01.txt\n");
	 //  break;

	case 13:
	break;
	  
	default:
	  Serial.print(F("\terror\n"));
	  break;
	}
  return 1;
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}


void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
	Serial.print('0');
  Serial.print(digits);
}

void ol_timestamp(){
  // digital clock display of the time
  delay(50);
  ol_printDigits(0,day());
  delay(50);
  ol_printDigits('/',month());
  delay(50);
  ol_printDigits('/',year());
  delay(50);
  ol_printDigits(';',hour());
  delay(50);
  ol_printDigits(':',minute());
  delay(50);
  ol_printDigits(':',second());
  delay(50);
  ol_serial.print(';'); 
}


// logs event timestamp and position
void ol_logevent(int event)
{
	  ol_serial.listen();
	  digitalWrite(SW_TOL,LOW);   //ON
      delay(2000);
    LED_blink(event,200);
	  ol_timestamp();
	  delay(100);
	  ol_serial.print("E");
	  ol_serial.print(event);
	  ol_serial.print(";");
	  delay(20);
	  ol_serial.print(latitude,6);
	  delay(20);
	  ol_serial.print(';');
	  ol_serial.print(longitude,6);
	  delay(20);
	  ol_serial.print(';');
	  delay(20);
	  ol_serial.print(temperature,3);
	  ol_serial.println("");
	  delay(4000);
	  digitalWrite(SW_TOL,HIGH); // OFF
}


// separator = 0 --> doesn't print
void ol_printDigits(char separator, int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  if (separator) ol_serial.write(separator);
  if(digits < 10)
	ol_serial.print('0');
  delay(10);
  ol_serial.print(digits);
}

int ol_cmd(){
  char temp;
  digitalWrite(SW_TOL, LOW);
  delay(1000);
  Serial.print(digitalRead(SW_TOL),DEC);
  Serial.print(F("Open command mode\n"));
  ol_serial.listen();
  delay(50);
  for (int i=0; ; i++){
    delay(1);
    ol_serial.write(26);
    delay(1);
    ol_serial.write(26);
    delay(1);
    ol_serial.write(26);
    delay(10);
    for (int z=0; z<10 ; z++)
      if (ol_serial.available())
        //Serial.println(ol_serial.read());
        if(ol_serial.read()=='>')
        { Serial.print("attempt: "); Serial.print(i, DEC); Serial.println(">");   delay(5); 
          ol_serial.println("");
          for (int j=0; j<200; j++)          //flush
            if (ol_serial.available())
              {temp=ol_serial.read(); 
              //Serial.print(temp);
              }
          delay(5);
          Serial.println("");
          Serial.println(F("OL CMD_MODE"));
          ol_status=1; 
          return 1;
        }
      delay(5);
      if (i>=250) { Serial.println(F("error openlong cmd")); return 0;}
    }     
}


int ol_append(){
//  digitalWrite(SW_TOL, HIGH);
//  ol_serial.listen();
//   for (int z=0; z<10 ; z++)
//   if (ol_serial.available())
//          //Serial.println(ol_serial.read());
//          if(ol_serial.read()=='>' || ol_serial.read()=='<')
//            {ol_status=0; Serial.println("Openlong back to CMD");}
  if (ol_status!=2)
  {
	  ol_cmd();
	  ol_serial.println("");
	  delay(10);
	  ol_serial.print("append log.txt");
	  ol_serial.write(13);
	  delay(10);
	  Serial.println(F("\tOpenlong ready"));
	  ol_status=2;
	  return 1;
  }
}
int ol_potbot_parameters(int num_parameters){
  int  par_index=0;  
  char character;
  int temp=0;
  ol_cmd();
  delay(3000);
  ol_serial.println(F("read potbot"));
  for (int i=0; i< 200; i++) {
  if (ol_serial.available()) 
  {
    character= ol_serial.read(); //Serial.write(character);
    Serial.print(character);    //uncomment to see openlog parameters load
    if (par_index>num_parameters) //end of line
    {
    Serial.println(F("\tparameters loaded"));  ol_serial.println("reset"); return 1; break;
    }   //LOW
    if ((character>= '0') &&  (character<= '9'))
      temp= temp*10 + character - '0';
    else if (character==',')
    {
      parameters[par_index]=temp;
      par_index ++;
      temp=0;
      Serial.print("\t   paramter[");
      Serial.print(par_index-1);
      Serial.print("] :");
      Serial.println(parameters[par_index - 1]);
    }
   }
   delay(10);
  }
   Serial.println(F("\tError OL_parameters"));
   digitalWrite(SW_TOL, HIGH);
   return 0;
}

byte sequence(byte state){
  static time_t temp1, temp2;
  
  switch(state){
	case 0:   // begin
	  ///////////
		command('C');
		command('G');
	  state=10;
	break;
	
	case 10:
	  delay(50);
	  Serial.println(F("E1 Wait WS"));
	  ol_logevent(1);  
	  delay(50);
	  state=1;
	break;
	case 1:   // not in water
	  sleep_ex();   //sleeps completely until wet switch active
	  state=20;
	break;

	case 20:
		delay(50);
	  Serial.println(F("E2 Deployed"));
	  Serial.println(F("\tVideo for P1"));
	  ol_logevent(2);
	  delay(50);
	  state=2;
	  temp1=now() + parameters[1];    // camera time
	  command('c');  //Cam on
	break;
	case 2:     // In water no video yet
	  if  (now()>temp1)     //Cam off
		{
			command('C');
	  	state=30;      
	  	delay(20);
	  }
	break;

	case 30:
		  /////////
		delay(50);
	  Serial.println(F("\nE3 In water - waiting"));
	  Serial.println(F("\t Tilted? wet?"));
	  ol_logevent(3);
	  delay(50);
	  state=3;
	  break;
	case 3:     // In water no video yet
	  if  (now()>temp2)     //store temp
		{
		  int val,frac;
		  temperature=read_temp(&val,&frac);
		  delay(100);
		  ol_logevent(3);
		  delay(100);  
		  temp2=now() + parameters[2];    // temperature time
		}
	  ///////////  TEST magnetometer  and go to 4
	  if  (mag_threshold(parameters[3], parameters[4]))     //angle +-10
		{
		  Serial.println("\tIn angle");
		  state=40;
		  delay(20);
		}
	  ///////////  TEST Wetswitch in AIR and go to 5
	  if (digitalRead(I_HUM))
	  {
		delay(1000);
		if (digitalRead(I_HUM))
		{
		  Serial.println("\tIn air");
		  state=50;
		  delay(20);
		}
	  }
	break;

	case 40:
		delay(50);
	  Serial.println(F("E4 Retrieving"));
	  Serial.println(F("\t wait for WS=air"));
	  ol_logevent(4);
	  delay(50);
	  state=4;
	  command('c');  //Cam on
	  temp1=now() + parameters[5];    // camera time
	  break;
	case 4:     // In water no video yet
	  if  (now()>temp1)     //Cam off
			{
				command('C');
				state=30;
			}
///////////  TEST Wetswitch in AIR and go to 5
	  if (digitalRead(I_HUM))
	  {
		delay(1000);
		if (digitalRead(I_HUM))
		{
		  Serial.println("\tIn air");
		  state=50;
		  delay(20);
		}
	  }
	break;

	case 50:
		delay(50);
	  Serial.println(F("E5 Retrieved"));
	  Serial.println(F("\t is wet?"));
	  ol_logevent(5);
	  delay(50);
	  state=5;
	  command('g'); //gps on
	  temp1=now() + parameters[6];    // GPS TIME
	  command('C'); //camera off
	break;
	
	case 5:     // Retrieved
	  
	  if  (now()>temp1)     
			{
				command('G');   //GPS off
				if (digitalRead(I_HUM))
			  {
				delay(1000);
				if (digitalRead(I_HUM))
				{
				  Serial.println(F("\tIn air"));
				  state=60;
				  delay(20);
				}
			  }
			}
			else
				get_gps(&latitude, &longitude);
	  ///////////  TEST Wetswitch in WATER and go to 2
	  if (!digitalRead(I_HUM))
	  {
			delay(1000);
			if (!digitalRead(I_HUM))
			{
			  Serial.println(F("\tWS in water"));
			  state=20;
			  delay(20);
			}
	  }
			///////////  TEST magnetometer  and go to 4
	  
	break;

	case 60:
		delay(50);
	  Serial.println(F("E6 Pre-deploy"));
	  Serial.println(F("\t Tilted? wet?"));
	  ol_logevent(6);
	  delay(50);
  	  temp1=now() + parameters[10];    // e6 TIME
	  state=6;
	break;

	case 6:     //2nd deployment?"
	  if  (mag_threshold(parameters[7], parameters[8]))     //angle +-10
		{
		  Serial.println(F("\tIn angle"));
		  state=70;
		  delay(20);
		}
	  ///////////  TEST Wetswitch in WATER and go to 2
	  else if (!digitalRead(I_HUM))
	  {
		delay(1000);
		if (!digitalRead(I_HUM))
		{
		  Serial.println(F("\tWS in water"));		delay(20);
		  state=20;
		  delay(20);
		}
	  }
  	  else if  (now()>temp1)     // no tilt or wet
		{
			command('G');   //GPS off
			command('C');   //Camera off
			Serial.println(F("\tWS in water"));		delay(20);
			state=0;   // go to begining 
			delay(20);
		}
	break;

	case 70:
		delay(50);
	  Serial.println(F("E7 Deploy"));
	  Serial.println("\t wet?");
	  ol_logevent(7);
	  delay(50);
	  state=7;
	  command('g');
	  temp1=now() + parameters[9];    // p9
	  command('C'); //camera off
	break;

	case 7:     // deploy?"
	  
	  if  (now()>temp1) 
	  {
	  command('G');
		Serial.println(F("\tGPS time out"));
		state=60;
		delay(20);
	  }
	  else
	  	get_gps(&latitude, &longitude);
	  ///////////  TEST Wetswitch in WATER and go to 2
	  if (!digitalRead(I_HUM))
	  {
		delay(1000);
		if (!digitalRead(I_HUM))
		{
		  Serial.println(F("\tWS in water"));
		  state=20;
		  delay(20);
		}
	  }
	break;

	default:
	state=0;
	break; 
  }
  return state;
}



// int sequence2(){
//   static time_t temp1, temp2;
  
//   switch(state){
//   case 0:
//     Serial.println("waiting acc ...");
//     if (get_acc_threshold(0))       // wait acc
//     {  Serial.print("wait for gps...");
//     state++;
//     }
//     delay (50);
//   break;

//   case 1:
//     ol_logevent(2);
//     temp1=now() + 60*parameters[4];    // gps time
//     temp2=now() + 60*parameters[1];    // camera time
//     command('g');   //start gps
//     command('c');   //start cam
//     state++;
//   break;
  
//   case 2:               // wait for gps
//     if (get_gps(&latitude, &longitude))
//     {
//       temp1=now() + 60*parameters[4];    // gps time
//       temp2=now() + 60*parameters[1];    // camera time
//       Serial.print("check deployment...");
//       ol_logevent(3);
//       state=22;
//     } 
	
//     else if (now()>temp1)     //gps time
//     {
//       ol_logevent(4);           // in water
//       state++;        //to never got signal
//     }
//   break;

//    case 3:           //turn off GPS if not recieving info in 1 sec, save last gps position after get into water
//     if  (!(get_gps(&latitude, &longitude)))
//     {
//       delay(100);
//       ol_logevent(5);
//       delay(100);
//       command('G'); //turn off gps
//       temp1=now() + 60*parameters[2];   // set time for temperature
//       Serial.print("In water... monitor temp");
//       state++;      // goto in water
//     } 
//     else if  (now()>temp1)     //potbot still in air after gps time
//     {

//       Serial.print("false deployment...");
//       command('G'); //turn off gps    //false deployment goto init
//       command('C'); //turn off camera
//       ol_logevent(99);  //False deployment
//     state=0;
//     }
//     break;
  
//   case 4:         // in water
//   int val,frac;
//     if  (now()>temp2)     //turn off camera after camera time
//       {ol_logevent(88); //Turn off camera after time elapsed
//       command('C');
//       temp2=now()+20*60*60;}
	  
//     if  (now()>temp1)     //read temperature every temp time
//     {
//       temperature=read_temp(&val,&frac);
//       delay(100);
//       ol_logevent(6);
//       delay(100);
//       temp1=now() + 60*parameters[2];   // set new time for temperature
//     }
	
//     if (get_acc_threshold(0))       // scan acc
//     {
//       temp1=now() + 60*parameters[4];    // gps time
//       command('g'); //turn on gps    
//       command('c'); //turn on camera
//       Serial.print("check pulled?...");
//       ol_logevent(7);
//     state++;  
//     }
//     break;

//   case 5:   // test pulled
//     if (get_gps(&latitude, &longitude))
//     {
//       command('C'); //turn off camera
//       get_gps(&latitude, &longitude);  
//       ol_logevent(8);
	  
//       temp1=now() + 60;   // set new time for transition 30'
//       Serial.print("transition to air test...");
//     state++;          // go to transition
//     } 
	
//     if (now()>temp1)        //check gps time
//     { Serial.println("false pulled");    //false pulled
//       command('G');                // turn off gps
//       command('C'); //turn off camera
//       ol_logevent(9);   //False pulled 
//       state =4;       // go to in water 
//     } 
//   break;
  
//   case 6:   // transition to in air
//     if (!(get_gps(&latitude, &longitude)))
//     {state=4; Serial.println("false pulled from transition");}  // false pulled from transition
//     else
//       if (now()>temp1)
//         {      ol_logevent(10);state=0; Serial.println("pulled confirmed, in air...");   // go to in air
//         command('G');
//       }  
//   break;  
  
//   case 10:
//     temp1=now()+20;
//     command('c');
//     state++;
//   break;

//   case 22:
	
//     if (now()>temp1) 
//     {
//       state=3;
//     }
//   break;

//   case 30:
//       Serial.println("state 2...");
//       get_gps(&latitude, &longitude);
//       digitalClockDisplay();
	
//     default:
	
//     break; 
//   }
// }
//Mag threshold + - 10
bool mag_threshold(int value, int axis)
{
  int mag_min,mag_max,mag_val;
  mag_val=get_mag(parameters[4], 1, 0);  
  mag_min=(value-10)*100;
  mag_max=(value+10)*100;
  Serial.print("\tmin, val, max  ");
  Serial.print(mag_min);Serial.print("  ");Serial.print(mag_val);Serial.print("  ");Serial.print(mag_max);Serial.print("\n");
  if  ((mag_val>=mag_min) && (mag_val<=mag_max)) 
		return 1;
  return 0;
}

int get_acc_threshold(bool serial_print){
  // sensors_event_t event;
  // bool acc_flag[3]={0,0,0};
  // float acc_avg[3]={11,11,11};
  // float acc_new[3];
  // byte acc_ch[3]={A0,A1,A2};

  //  delay(5);
  // for (int i=0; i<3;i++)
  // {accel.getEvent(&event); delay(5);}
  //   accel.getEvent(&event);
  //   acc_avg[0]=accel.raw.x + ACC_OFFSET;
  //   acc_avg[1]=accel.raw.y + ACC_OFFSET;
  //   acc_avg[2]=accel.raw.z + ACC_OFFSET;

  // for (unsigned long start = millis(); millis() - start < 1000;)   //check for 1 seconds
  // {
  //   if ((int)acc_flag[0]+ (int)acc_flag[1]+ (int)acc_flag[2]>=2) {Serial.println("\n Accelerometer threshold passed"); return 1;}
  //   for (int i=0; i<3;i++)
  //   {
  //   accel.getEvent(&event);
  //   acc_new[0]=accel.raw.x + ACC_OFFSET;
  //   acc_new[1]=accel.raw.y + ACC_OFFSET;
  //   acc_new[2]=accel.raw.z + ACC_OFFSET;

  //   if ((acc_new[i]> ((100+(float)parameters[3])*acc_avg[i])/100) || (acc_new[i]< ((100-parameters[3])*acc_avg[i])/100))   // set flag and check
  //   {
  //     delay(20);
  //     accel.getEvent(&event);
  //     acc_new[0]=accel.raw.x + ACC_OFFSET;
  //     acc_new[1]=accel.raw.y + ACC_OFFSET;
  //     acc_new[2]=accel.raw.z + ACC_OFFSET;
  //   if ((acc_new[i]> ((100+(float)parameters[3])*acc_avg[i])/100) || (acc_new[i]< ((100-parameters[3])*acc_avg[i])/100))
  //     { acc_flag[i]=1;Serial.print("ch:"); Serial.print(i);Serial.print("  acc flag avg: "); Serial.print((acc_avg[i])); Serial.print("  val: "); Serial.println((acc_new[i])); }
  //     else
  //     {acc_flag[0]=acc_flag[0];}  //dummy
  //     }
  //   else
  //   {
  //   acc_avg[i]=acc_avg[i]/2;
  //   acc_avg[i]=acc_avg[i] + acc_new[i]/2;
  //   delay(5);
  //   if(serial_print) {Serial.print("avg ch:");Serial.print(i);Serial.print("val :");Serial.print(acc_avg[i],DEC); Serial.print(";");}       
  //   }
  // acc_avg[i]=acc_new[i];
  //   }
  //   if(serial_print) Serial.println("");
  // }
  return 0;
}

void sleep_ex()
{
  delay(1000);
  Serial.println("\tEX_Sleep");
  delay(25);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
 
	sleep_enable();          // enables the sleep bit in the mcucr register
	attachInterrupt(0,pin2_isr, FALLING); // use interrupt 0 (pin 2) and run functionwakeUpNow when pin 2 gets LOW
	sleep_mode();            // here the device is actually put to sleep!!THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
	cli();
	sleep_disable();         // first thing after waking from sleep:                             // disable sleep...
	detachInterrupt(0);      // disables interrupt 0 on pin 2 so the  // wakeUpNow code will not be executed/ during normal running time.
  sei();
}

void pin2_isr()
{
  
}

void sleep_WD() {  // turn on watchdog timer; interrupt mode every 2.0s
  //    // WDTCSR    // 4 last bits 1001-->8s, 1000->4s, 0111->2s, ,  0110-> 1s
  // Serial.println("\tWD_sleep");
  delay(25);
  cli();
  MCUSR = 0;
  WDTCSR |= B00011000;     
  WDTCSR =  B01000111;     //2s
  sei();
  sleep_mode();
}

ISR(WDT_vect) {
 cli();
 wdt_disable();
 setTime(now()+2); // corect time
 // Serial.println("wakeup!");
 sei();
}

void LED_blink(int blinks, unsigned long delay_time)
{
   for (int i=0; i<blinks; i++)
   {
    digitalWrite(LED, HIGH);
    delay(delay_time);
    digitalWrite(LED, LOW);
    delay(delay_time);
  }
}

// OL CONFIG
// 4800,26,3,1,1,1,0
// baud,escape,esc#,mode,verb,echo,ignoreRX
