#include <Wire.h>
#include <LPS.h>
#include <LSM6.h>
#include "SparkFunLSM6DS3.h"
#include <LIS3MDL.h>
#include <SD.h>
#include <Servo.h>
#include <Math.h>
#include "SensorFusion.h"

const int led = LED_BUILTIN;
int ledIter=0;
LPS ps;
LSM6DS3 imu;
LIS3MDL mag;
SF ahrs;
//char report[80];
const int chipSelect =  BUILTIN_SDCARD;
Servo fin1; Servo fin2; Servo fin3;

const int desiredDelay=50; //20 Hz
String filename;

// controls
enum bfr_state{IDLE,PAD,LAUNCH,FLIGHT};
enum bfr_state state = PAD;

float ax_0=0; float ay_0=0; float az_0=0;
#define s_0v 20
float gx_0v[s_0v]; float gy_0v[s_0v]; float gz_0v[s_0v]; uint8_t i_0v=0;
#define s_rv 20 
float gx_rv[s_rv]; float gy_rv[s_rv]; float gz_rv[s_rv]; uint8_t i_rv=0;
float gx_r=0; float gy_r=0; float gz_r=0;
float gx_0=0; float gy_0=0; float gz_0=0;

// timers
long int time_0=0; // initial time 
long int time_p=0; // previous step time
long int time_zero=0;
long int time_print=0;

//helper functions
void update_offsets();

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  Serial.begin(38400);
  Wire2.begin();

//  initialize Pressure
  if (!ps.init()) {
    while(1){Serial.println("Failed to autodetect pressure sensor!"); delay(10);};
  }
  ps.enableDefault();

//  initialize IMU
  if (imu.begin() != 0) {
    while(1){Serial.println("Failed to detect and initialize IMU!"); delay(10);};
  }
  imu.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  imu.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  imu.begin();
  
// initialize mag
  if (!mag.init()) {
    while(1){Serial.println("Failed to detect and initialize magnetometer!"); delay(10);};
  }
  mag.enableDefault();

// initialize servo
  fin1.attach(0); fin2.attach(0); fin3.attach(0); // TODO
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
      Serial.println("Card failed, or not present");
      delay(10);
    }
  } else {
    int i=0;
    filename="datalog_"+String(i)+".txt";
    while (SD.exists(filename.c_str())) {
      i+=1;
      filename="datalog_"+String(i)+".txt";
    }
//     Serial.println("Card initialized, logging to: "+ filename);
  }
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time [ms], p [mbar], alt [m], T [c], accel x [g], accel y [g], accel z [g], gyro x [deg/sec], gyro y [deg/sec], gyro z [deg/sec]"  );
    dataFile.close();
//    Serial.println("Time [ms], p [mbar], alt [m], T [c], accel x [g], accel y [g], accel z [g], gyro x [deg/sec], gyro y [deg/sec], gyro z [deg/sec]");
  } else {
//    while (1){Serial.println("error opening datalog.txt"); delay(10);}
  }

  time_0=millis(); time_p=millis();
}


void loop() {
  // // // // // // // // //
  // READ SENSORS
  // // // // // // // // //
  float pressure = ps.readPressureMillibars(); float altitude = ps.pressureToAltitudeMeters(pressure); float temperature = ps.readTemperatureC();
  float ax_r=imu.readFloatAccelX(); float ay_r=imu.readFloatAccelY(); float az_r=imu.readFloatAccelZ(); 
  // float gx_r=imu.readFloatGyroX();  float gy_r=imu.readFloatGyroY();  float gz_r=imu.readFloatGyroZ();
  float ax=ax_r-ax_0; float ay=ay_r-ay_0; float az=az_r-az_0; 
  // smooth out noisy gyro measurements
  gx_rv[i_rv]=imu.readFloatGyroX(); gy_rv[i_rv]=imu.readFloatGyroY(); gz_rv[i_rv]=imu.readFloatGyroZ(); i_rv=(i_rv+1)%s_rv;
  gx_r=0; gy_r=0; gz_r=0;
  // average across array
  for(uint8_t i=0; i<s_rv; i++) {
    gx_r+=gx_rv[i]; gy_r+=gy_rv[i]; gz_r+=gz_rv[i];
  }
  gx_r/=s_rv; gy_r/=s_rv; gz_r/=s_rv;
  float gx=gx_r-gx_0; float gy=gy_r-gy_0; float gz=gz_r-gz_0;

  mag.read();
  float mx=mag.m.x; float my=mag.m.y; float mz=mag.m.z;

  float dt = ahrs.deltatUpdate(); time_p = millis();
  ahrs.MadgwickUpdate(gx,gy,gz,ax,ay,az,mx,my,mz,dt);
  float* q = ahrs.getQuat();

  // // // // // // // // //
  // STATE TRANSITION LOGIC
  // // // // // // // // //
//  stop updating offset
  if (millis()-time_0 > 5000) {
    state = IDLE;
  }

  // // // // // // // // //
  // STATE IDLE
  // // // // // // // // //
  if (state == IDLE) {
//    if gyro small, remove
    if (sqrt(gx*gx+gy*gy+gz*gz) < 10) {
      update_offsets();
    }
  }

  // // // // // // // // //
  // STATE PAD
  // // // // // // // // //
  else if (state == PAD) {
    update_offsets();
    
  }

  // // // // // // // // //
  // STATE LAUNCH
  // // // // // // // // //
  else if (state == LAUNCH) {
    
  }

  // // // // // // // // //
  // STATE FLIGHT
  // // // // // // // // //
  else if (state == FLIGHT) {
    
  }

  // // // // // // // // //
  // LOG DATA
  // // // // // // // // //
  if (millis()-time_print > 100) { time_print = millis();
//    String ds = "ACC "+String(ax)+" "+String(ay)+" "+String(az)+" GYR "+String(gx)+" "+String(gy)+" "+String(gz)+" Q "+String(q0)+" "+String(q1)+" "+String(q2)+" "+String(q3);
    String ds = String(gx)+" "+String(gy)+" "+String(gz);//+" "+String(gx_r)+" "+String(gy_r)+" "+String(gz_r)+" "+String(gx_0)+" "+String(gy_0)+" "+String(gz_0);
//    String ds = String(q[0])+" "+String(q[1])+" "+String(q[2])+" "+String(q[3]);
//    String ds = String(mag.m.x)+" "+String(mag.m.y)+" "+String(mag.m.z);
//    String ds = String(ax)+" "+String(ay)+" "+String(az);
//    String ds = String(ahrs.getRoll())+" "+String(ahrs.getPitch())+" "+String(ahrs.getYaw());
    Serial.println(ds);
  }

}

// // // // // // // // //
// HELPER FUNCTIONS
// // // // // // // // //
void update_offsets() {
  gx_0v[i_0v]=gx_r; gy_0v[i_0v]=gy_r; gz_0v[i_0v]=gz_r; i_0v=(i_0v+1)%s_0v;
  gx_0=0; gy_0=0; gz_0=0;
  // average across array
  for(uint8_t i=0; i<s_0v; i++) {
    gx_0+=gx_0v[i]; gy_0+=gy_0v[i]; gz_0+=gz_0v[i];
  }
  gx_0/=s_0v; gy_0/=s_0v; gz_0/=s_0v;
}


/*
void loop() {
  // put your main code here, to run repeatedly:
  long int t1=millis();
  if ((ledIter<=25 && (t1-time_0)<5000) ){//|| (ledIter<=150 && (t1-time_0)>=5000)) {
      ledIter+=1;
  }else if ( (t1-time_0)>=5000){
      ledIter=0;
  }else{
    digitalWrite(led, HIGH);
    ledIter=0;
  }
  String dataString="";
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();
//  dataString="Time [ms]: "+ String(t1-time_0) +", p [mbar]: " + String(pressure) + ", alt [m]: "+ String(altitude) +", T [c]: " + String(temperature);
//  dataString=String(t1-time_0) +", " + String(pressure) + ", "+ String(altitude) +", " + String(temperature);
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
*/
