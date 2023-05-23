#include "ahrs.h"
#include <Wire.h>
#include <LPS.h>
#include <LSM6.h>
#include "SparkFunLSM6DS3.h"
#include <LIS3MDL.h>
#include <SD.h>

const int led = LED_BUILTIN;
int ledIter=0;
LPS ps;
LSM6DS3 imu;
LIS3MDL mag;
//char report[80];
const int chipSelect =  BUILTIN_SDCARD;
const int desiredDelay=50; //20 Hz
long int t0;
String filename;

long int lastTime = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  Serial.begin(38400);
  Wire2.begin();

//  initialize Pressure
  if (!ps.init())
  {
    while(1){Serial.println("Failed to autodetect pressure sensor!"); delay(10);};
  }
  ps.enableDefault();

//  initialize IMU
  if (imu.begin() != 0)
  {
    while(1){Serial.println("Failed to detect and initialize IMU!"); delay(10);};
  }
  imu.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  imu.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  imu.begin();
  
// initialize mag
 if (!mag.init())
  {
    while(1){Serial.println("Failed to detect and initialize magnetometer!"); delay(10);};
  }

  mag.enableDefault();


  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
      Serial.println("Card failed, or not present");
      delay(10);
    }
  }else{
    int i=0;
    filename="datalog_"+String(i)+".txt";
    while (SD.exists(filename.c_str())){
      i+=1;
      filename="datalog_"+String(i)+".txt";
    }
     Serial.println("card initialized, logging to: "+ filename);
  }
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time [ms], p [mbar], alt [m], T [c], accel x [g], accel y [g], accel z [g], gyro x [deg/sec], gyro y [deg/sec], gyro z [deg/sec]"  );
    dataFile.close();
    Serial.println("Time [ms], p [mbar], alt [m], T [c], accel x [g], accel y [g], accel z [g], gyro x [deg/sec], gyro y [deg/sec], gyro z [deg/sec]");
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
    while (1){Serial.println("error opening datalog.txt");delay(10);}
  }
//  Serial.println(F_CPU_ACTUAL);
//  dataFile = SD.open("datalog.txt", FILE_WRITE);
  t0=millis();
  lastTime = millis();
}
  


void loop() {
  long int t1=millis();
  if ((ledIter<=25 && (t1-t0)<5000) ){//|| (ledIter<=150 && (t1-t0)>=5000)) {
      ledIter+=1;
  }else if ( (t1-t0)>=5000){
      ledIter=0;
  }else{
    digitalWrite(led, HIGH);
    ledIter=0;
  }
  String dataString="";
  // put your main code here, to run repeatedly:
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();
//  dataString="Time [ms]: "+ String(t1-t0) +", p [mbar]: " + String(pressure) + ", alt [m]: "+ String(altitude) +", T [c]: " + String(temperature);
//  dataString=String(t1-t0) +", " + String(pressure) + ", "+ String(altitude) +", " + String(temperature);
  float ax=imu.readFloatAccelX();
  float ay=imu.readFloatAccelY();
  float az=imu.readFloatAccelZ();
  float gx=imu.readFloatGyroX();
  float gy=imu.readFloatGyroY();
  float gz=imu.readFloatGyroZ();
    mag.read();

  dataString=dataString+", accel [g]: " + String(ax) + " " + String(ay) + " " + String(az);
  dataString=dataString+ ", gyro [deg/sec]: " + String(gx) + " " + String(gy) + " " + String(gz);
//  dataString=dataString+", mag: " + String(mag.m.x) + " " + String(mag.m.y) + " " + String(mag.m.z);
//  dataString=dataString+", " + String(ax) + ", " + String(ay) + ", " + String(az);
//  dataString=dataString+ ", " + String(gx) + ", " + String(gy) + ", " + String(gz);

float dt = (millis() - lastTime)/1000.0; lastTime=millis();
AHRSupdate(dt,gx,gy,gz,ax,ay,az,float(mag.m.x),float(mag.m.y),float(mag.m.z));
dataString=dataString+" q: "+" "+String(q0)+" "+String(q1)+" "+String(q2)+" "+String(q3);

  // open the file.
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
  long int t2=millis();
  int delta=(t2-t1);
  if (desiredDelay > delta){
//    Serial.println(delta);
    delay(desiredDelay-delta); // run at a reasonable not-too-fast speed
  } else {
     Serial.println("Violated Hz.");
  }
  if (ledIter==0){
    digitalWrite(led, LOW);
  }
}
