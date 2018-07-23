#include "Time.h"
#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

#include <SoftwareSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include <TinyGPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>


#define EEPROM_addrs  1
#define SW_ACC  2
#define SW_TOL  3
#define SW_GPS  4
#define SW_CAM  5
#define VI_ACC  A1
#define TX_GPS  13
#define RX_GPS  12
#define RX_OL  11
#define TX_OL  10
#define LED  13
#define I_HUM  A3
#define temp_add 72
#define GPS_THRESHOLD 3
#define ACC_OFFSET 15000 // all positives values

SoftwareSerial gps_serial(RX_GPS, TX_GPS); // RX, TX   // GPS  12 13
SoftwareSerial ol_serial(RX_OL, TX_OL); // RX, TX   // OL
//time
TinyGPS gps;
byte ol_status=0;   //0 nothing    1 CMD mode    2 Append
bool first;
bool screen=0;
bool logger=0;
static bool gps_write=0;
char gps_thresh=0;
float latitude, longitude, temperature;
byte state=0;    // 1: in air    3: deployed    5: in water    7: recovered
//dummy    ,Video_time [min], temp_freq [min], acc_threshold [%], GPS_time out [min]
byte parameters[5]={0,15,60,20,5};

float read_temp(int *val, int *frac);
void log_acc();
int get_gps(float *latitude, float *longitude);
int command(char data);
void digitalClockDisplay();
void printDigits(int digits);
int ol_cmd();
int ol_append();
int ol_potbot_parameters();
int get_acc_threshold(bool serial_print);
int sequence();
void ol_timestamp();
void ol_printDigits(char separator, int digits);
void ol_logevent(int event);
//Accelerometer config
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void setup()
{

  Wire.begin();
  // initialize the serial communication:
  Serial.begin(4800);
  // initialize 
  pinMode(SW_ACC, OUTPUT);
  pinMode(SW_TOL, OUTPUT);
  pinMode(SW_GPS, OUTPUT);
  pinMode(SW_CAM, OUTPUT);
  pinMode(VI_ACC, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(I_HUM, INPUT);

  digitalWrite(SW_ACC, LOW);
  
  gps_serial.begin(4800); 
  ol_serial.begin(4800);
  delay (1000);
    
  if(Serial)
    Serial.println("PotBot v_2.5");
  
  delay(1000);
  ol_potbot_parameters();
  
  state=0;

    /* Initialise the accelerometer */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
   /* Enable auto-gain  Magnetometer*/
  mag.enableAutoRange(true);
  ol_logevent(0);
}

void loop() {
  char data;
        digitalWrite(SW_GPS, HIGH);
    get_gps(&latitude, &longitude);
/*   if (screen)                        // OPENLONG SCREEN INTERFACE
       {  if (ol_serial.available()) {
        Serial.write(ol_serial.read());
        }
        if (Serial.available()) {
        ol_serial.write(Serial.read());
         }
      }
  else {
          // DEBUG interface
      if (Serial.available()){
      data   = Serial.read();
      Serial.write(data);
      Serial.print("   :");
      command(data);
      }
    }
  
  if (logger)   //LOGGER
  {
    log_acc();
  }   
  //state=1;
  sequence(); */
}


float read_temp(int *val, int *frac){
  float temperature;
  digitalWrite(SW_TOL, HIGH);
  digitalWrite(LED, HIGH);
  Serial.print("TEMP ON \n");  //ZZZ delete
  delay(10);
   Serial.print("begin tx \n");  //ZZZ delete
  Wire.beginTransmission(temp_add);
  // send configuration
  Serial.print("send config \n");  //ZZZ delete
  Wire.write(0xAC);
  Wire.write(B00001111); // 12 bit resolution, pol, oneshot
  Wire.endTransmission();
  delay(10);
 
  // begin convert
   Serial.print("begin convert \n");  //ZZZ delete
  Wire.beginTransmission(temp_add); 
  Wire.write(0x51);
  Wire.endTransmission(); 
  delay(20);
 
  // wait until converting is done
   Serial.print("Wait for convertion \n");  //ZZZ delete
  byte conf = 0;
  while ( conf & B1000000 != B10000000 ) {
    delay(20);
    Wire.beginTransmission(temp_add); 
    Wire.write(0xAC); 
    Wire.endTransmission();
    conf = Wire.read();
  }

  Serial.print("Ask temperature \n");  //ZZZ delete
  // ask for the temerature 
  Wire.beginTransmission(temp_add); 
  Wire.write(0xAA); 
  Wire.endTransmission();

  Serial.print("Read bytes \n");  //ZZZ delete
  // request 2 bytes
  Wire.requestFrom(temp_add, 2);
  // read first byte
 
  *val = Wire.read();
  
  // read second byte
  *frac = Wire.read();
  
  if (*frac>100)
    temperature=(float)*frac/1000;
  else if (*frac>10)
    temperature=(float)*frac/100;
  else
    temperature=(float)*frac/10;
  temperature+=(float)*val;
  Serial.print("\n  Temperature float: ");
  Serial.println(temperature,3);
  *frac = 100 * (*frac & 0xF0 )/ 256;
  if ( *frac < 10 ) {

  }

  Serial.print("\n");

  delay(50);
  digitalWrite(SW_TOL, HIGH);  //LOW
  digitalWrite(LED, LOW);
  return temperature;
}

void log_acc(){
    delay(50);
    sensors_event_t event_acc;
    accel.getEvent(&event_acc);
    sensors_event_t event_mag;
    mag.getEvent(&event_mag);
    //Serial.print("logging... \n");  //ZZZ delete

    ol_serial.print(event_acc.acceleration.x);
    ol_serial.print(";");
    ol_serial.print(event_acc.acceleration.y);
    ol_serial.print(";");
    ol_serial.print(event_acc.acceleration.z);
    ol_serial.print(";");
    ol_serial.print(event_mag.magnetic.x);
    ol_serial.print(";");
    ol_serial.print(event_mag.magnetic.y);
    ol_serial.print(";");
    ol_serial.print(event_mag.magnetic.z);
    ol_serial.println("");
}
/// get_gps:
//Read GPS for a second and 
//return 1: valid sentence recieved 
//return 0: no valid
// latitude and longitude as floats
int get_gps(float *latitude, float *longitude)
{
	gps_serial.listen();
	Serial.println("GPS Start");
   extern bool gps_write;
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
     // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
     if (gps.encode(c)) // Did a new valid sentence come in?
     {
      newData = true;
      gps_thresh++;
     }
     }
   }
   Serial.println("Timed out");

   if (newData && gps_thresh>GPS_THRESHOLD) // threshold # of good sentences
   {
  unsigned long age;
  gps.f_get_position(latitude, longitude, &age);
  gps.crack_datetime(&year, &month, &day,
  &hour, &minutes, &second, &hundredths, &fix_age);
  setTime(hour,minutes,second,day,month,year);
  setTime(now()+28800);
  gps_thresh=0;
  //Serial.println("Lat: %f %f",latitude,longitude);
  Serial.println("GPS received");
  return 1;
   }
return 0;
  
 }

int command(char data){
  switch (data){
    case 'a': 
      digitalWrite(SW_ACC, !digitalRead(SW_ACC));
      Serial.print(digitalRead(SW_ACC),DEC);
      Serial.print("  ACC\n");
      
            
      break;

    case 't': 
      digitalWrite(SW_TOL, !digitalRead(SW_TOL));
      Serial.print(digitalRead(SW_TOL),DEC);
      Serial.print("  Temp & Open log\n");
    int val, frac;
      temperature=read_temp( &val, &frac);
      
      break;

    case 'T': 
      digitalWrite(SW_TOL, LOW);
      Serial.print(digitalRead(SW_TOL),DEC);
      Serial.print(" LOGGER OFF \n");
      break;

    case 'o': 
      //ol_cmd();
      break;

    case 'g': 
      digitalWrite(SW_GPS, HIGH);
      Serial.println("GPS on...");
      gps_serial.listen();
  break;
  
  case 'G': 
      digitalWrite(SW_GPS, LOW);
      Serial.println("GPS off...");
  break;
      
    case 'c':
      digitalWrite(SW_CAM, HIGH);
      Serial.print("  Camera on \n");
  break;

  case 'C':
    if(digitalRead(SW_CAM))
      {digitalWrite(SW_CAM, LOW);      Serial.print("  Camera off \n");}
  break;
  
    case 'h':
      Serial.print(digitalRead(I_HUM),DEC);
      Serial.print("  hum\n");
      break;


    case 'w':
      digitalWrite(SW_TOL, HIGH);
      Serial.print("Open log writing test\n");
      delay(1000);
      for(int i = 1 ; i < 10 ; i++) {
      ol_serial.print(i, DEC);     
      ol_serial.println(":abcdefsdff-!#");
      delay(100);
      }
      
      delay(1000);
      Serial.print("finish writing\n");
      digitalWrite(SW_TOL, LOW);  

      break;

     case 's':                                   //screen
     digitalWrite(SW_TOL, HIGH);
     ol_serial.listen();
     ol_status=0;
     ol_cmd();
     state=100;
     screen=1;
     
     
     
     break;


     case 'l':                                   //log
     state=100;
     digitalWrite(SW_TOL, HIGH);
     digitalWrite(SW_ACC, LOW);
     delay(500);
      Serial.print("  Logger: ");
      Serial.print(!logger,DEC);
      Serial.println("");
      ol_serial.listen();
    
    if (!logger){
       for (int i=0; i<20 ; i++){
         ol_serial.println("");
         ol_serial.write(26);
         ol_serial.write(26);
         ol_serial.write(26);
         ol_serial.println("");
         if (ol_serial.available())
        //Serial.println(ol_serial.read());
          if(ol_serial.read()=='>')
            {Serial.print(i, DEC); Serial.println(">"); break;}
         }
       delay(20);
       int value = EEPROM.read(EEPROM_addrs);
       EEPROM.write(EEPROM_addrs, value+1);
       ol_serial.println("");
       delay(10);
       ol_serial.println("");
       delay(10);
       ol_serial.print("append mylog_");
       if (value<10) ol_serial.print("0");
       ol_serial.print(value,DEC);
       ol_serial.println(".txt");
       delay(10);
    }else
    {
         ol_serial.write(26);
         ol_serial.println("");
         delay(10);
    }
        ol_serial.println("x (m/s^2);y (m/s^2);z (m/s^2);x (uT);y (uT);z (uT)");
    logger=!logger;
     break;

     case 'r':
      EEPROM.write(EEPROM_addrs, 0);
      Serial.print("  reset logger file to new01.txt\n");
      break;

    case 13:
    break;
      
    default:
      Serial.print("  error\n");
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
      delay(100);
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
if (!ol_status)
{ 
  char temp;
  digitalWrite(SW_TOL, HIGH);
  Serial.print(digitalRead(SW_TOL),DEC);
  Serial.print("Open command mode\n");
  ol_serial.listen();
  delay(50);
  for (int i=0; i<51 ; i++){
      delay(5);
      ol_serial.write(26);
      ol_serial.write(26);
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
            Serial.println("OL CMD_MODE");
            ol_status=1; 
            return 1;
          }
        delay(5);
        if (i>=50) { Serial.println("error openlong cmd"); return 0;}
      }
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
  /*if (ol_status!=2)
  {
      ol_cmd();
      ol_serial.println("");
      delay(10);
      ol_serial.print("append log.txt");
      ol_serial.write(13);
      delay(10);
      Serial.println("Openlong ready");
      ol_status=2;
      return 1;
  }*/
}
int ol_potbot_parameters(){
  int  par_index=0;  
  char character;
  int temp=0;
  ol_cmd();
  delay(200);
  ol_serial.println("read potbot");
  for (int i=0; i< 200; i++) {
    if (ol_serial.available()) 
    {
      character= ol_serial.read(); //Serial.write(character);
      if (character=='.') {Serial.println("parameters loaded");  ol_serial.println("reset"); digitalWrite(SW_TOL, LOW);return 1; break;}   //LOW
        if ((character>= '0') &&  (character<= '9'))
          temp= temp*10 + character - '0';
        else if (character==',')
        {
          parameters[par_index]=temp;
          par_index ++;
          temp=0;
          Serial.print("    paramter[");
          Serial.print(par_index-1);
          Serial.print("] :");
          Serial.println(parameters[par_index - 1]);
        }
     }
     delay(10);
    }
   Serial.println("Error OL_parameters");
   digitalWrite(SW_TOL, LOW);
   return 0;
}

int sequence(){
  static time_t temp1, temp2;
  
  switch(state){
  case 0:
    Serial.println("waiting acc ...");
    if (get_acc_threshold(0))       // wait acc
    {  Serial.print("wait for gps...");
    state++;
    }
    delay (50);
  break;

  case 1:
    ol_logevent(2);
    temp1=now() + 60*parameters[4];    // gps time
    temp2=now() + 60*parameters[1];    // camera time
    command('g');   //start gps
    command('c');   //start cam
    state++;
  break;
  
  case 2:               // wait for gps
    if (get_gps(&latitude, &longitude))
    {
      temp1=now() + 60*parameters[4];    // gps time
      temp2=now() + 60*parameters[1];    // camera time
      Serial.print("check deployment...");
      ol_logevent(3);
      state++;
    } 
    
    else if (now()>temp1)     //gps time
    {
      ol_logevent(4);
      state++;        //to never got signal
    }
  break;

   case 3:           //turn off GPS if not recieving info in 1 sec, save last gps position after get into water
    if  (!(get_gps(&latitude, &longitude)))
    {
      delay(100);
      ol_logevent(5);
      delay(100);
      command('G'); //turn off gps
      temp1=now() + 60*parameters[2];   // set time for temperature
      Serial.print("In water... monitor temp");
    state++;      // goto in water
    } 
    else if  (now()>temp1)     //potbot still in air after gps time
    {
      command('G'); //turn off gps    //false deployment goto init
      command('C'); //turn off camera
      Serial.print("false deployment...");
    state=0;
    }
    break;
  
  case 4:         // in water
  int val,frac;
    if  (now()>temp2)     //turn off camera after camera time
      command('C');
      
    if  (now()>temp1)     //read temperature every temp time
    {
      temperature=read_temp(&val,&frac);
      delay(100);
      ol_logevent(6);
      delay(100);
      temp1=now() + 60*parameters[2];   // set new time for temperature
    }
    
    if (get_acc_threshold(0))       // scan acc
    {
      temp1=now() + 60*parameters[4];    // gps time
      command('g'); //turn on gps    
      command('c'); //turn on camera
      Serial.print("check pulled?...");
    state++;  
    }
    break;

  case 5:   // test pulled
    if (get_gps(&latitude, &longitude))
    {
      command('C'); //turn off camera
      get_gps(&latitude, &longitude);   // store 3rd value
      get_gps(&latitude, &longitude);

      ol_append();
      ol_timestamp();
      ol_serial.print("pulled;");
      ol_serial.print(latitude,6);
      ol_serial.print(';');
      ol_serial.print(longitude,6);
      ol_serial.println("");
      
      temp1=now() + 60*30;   // set new time for transition 30'
      Serial.print("transition to air test...");
    state++;          // go to transition
    } 
    
    if (now()>temp1)        //check gps time
    { Serial.println("false pulled");    //false pulled
      command('G');                // turn off gps
      state =4;       // go to in water 
    } 
  break;
  
  case 6:   // transition to in air
    if (!(get_gps(&latitude, &longitude)))
    {state=4; Serial.println("false pulled from transition");}  // false pulled from transition
    else
      if (now()>temp1)
        {state=0; Serial.println("pulled confirmed, in air...");   // go to in air
        command('G');
      }  
  break;  
  
  case 10:
    temp1=now()+20;
    digitalWrite(SW_CAM,HIGH);
    Serial.println("camera on...");
    state++;
  break;

  case 20:
    
    if (now()>temp1) 
    {
      Serial.println("camera off...");
      digitalWrite(SW_CAM,LOW); 
      state=0;
    }
  break;

  case 30:
      Serial.println("state 2...");
      get_gps(&latitude, &longitude);
      digitalClockDisplay();
    
    default:
    
    break; 
  }
}

int get_acc_threshold(bool serial_print){
  sensors_event_t event;
  bool acc_flag[3]={0,0,0};
  float acc_avg[3]={11,11,11};
  float acc_new[3];
  byte acc_ch[3]={A0,A1,A2};

   delay(5);
  for (int i=0; i<3;i++)
  {accel.getEvent(&event); delay(5);}
    accel.getEvent(&event);
    acc_avg[0]=accel.raw.x + ACC_OFFSET;
    acc_avg[1]=accel.raw.y + ACC_OFFSET;
    acc_avg[2]=accel.raw.z + ACC_OFFSET;

  for (unsigned long start = millis(); millis() - start < 5000;)   //check for 5 seconds
  {
    if ((int)acc_flag[0]+ (int)acc_flag[1]+ (int)acc_flag[2]>=2) {Serial.println("\n Accelerometer threshold passed"); return 1;}
    for (int i=0; i<3;i++)
    {
    accel.getEvent(&event);
    acc_new[0]=accel.raw.x + ACC_OFFSET;
    acc_new[1]=accel.raw.y + ACC_OFFSET;
    acc_new[2]=accel.raw.z + ACC_OFFSET;

    if ((acc_new[i]> ((100+(float)parameters[3])*acc_avg[i])/100) || (acc_new[i]< ((100-parameters[3])*acc_avg[i])/100))   // set flag and check
    {
      delay(20);
      accel.getEvent(&event);
      acc_new[0]=accel.raw.x + ACC_OFFSET;
      acc_new[1]=accel.raw.y + ACC_OFFSET;
      acc_new[2]=accel.raw.z + ACC_OFFSET;
    if ((acc_new[i]> ((100+(float)parameters[3])*acc_avg[i])/100) || (acc_new[i]< ((100-parameters[3])*acc_avg[i])/100))
      { acc_flag[i]=1;Serial.print("ch:"); Serial.print(i);Serial.print("  acc flag avg: "); Serial.print((acc_avg[i])); Serial.print("  val: "); Serial.println((acc_new[i])); }
      else
      {acc_flag[0]=acc_flag[0];}  //dummy
      }
    else
    {
    acc_avg[i]=acc_avg[i]/2;
    acc_avg[i]=acc_avg[i] + acc_new[i]/2;
    delay(5);
    if(serial_print) {Serial.print("avg ch:");Serial.print(i);Serial.print("val :");Serial.print(acc_avg[i],DEC); Serial.print(";");}       
    }
  acc_avg[i]=acc_new[i];
    }
    if(serial_print) Serial.println("");
  }
  return 0;
}



