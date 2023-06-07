#include <Wire.h>
#include <LPS.h>
#include <LSM6.h>
#include "SparkFunLSM6DS3.h"
#include <LIS3MDL.h>
#include <SD.h>
#include <Servo.h>
#include <Math.h>
#include "SensorFusion.h"
#include <quaternion_type.h>

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
enum bfr_state state = IDLE;

// PD controller
#define Kp 15
#define Kd 1
// mixer gains
#define Lx 3
#define Ly 3
#define Lz 2
// servo lims
#define contr_min -100
#define contr_max 100
#define servo_min 1000 // microsec
#define servo_max 2000

float L1, L2, L3;
uint16_t u1, u2, u3;

// q [1 0 0 0] is flat, starside down facing North (NED)
quat_t q_set = {1,0,0,0};

// AHRS
float ax_0=0; float ay_0=0; float az_0=0;
#define s_0v 128
float gx_0v[s_0v]; float gy_0v[s_0v]; float gz_0v[s_0v]; uint8_t i_0v=0;
#define s_rv 32 
float ax_rv[s_rv]; float ay_rv[s_rv]; float az_rv[s_rv];
float gx_rv[s_rv]; float gy_rv[s_rv]; float gz_rv[s_rv]; uint8_t i_rv=0;
float gx_r=0; float gy_r=0; float gz_r=0;
float gx_0=0; float gy_0=0; float gz_0=0;

// timers
long int time_0=0; // initial time 
long int time_p=0; // previous step time
long int time_zero=0;
long int time_print=0;
long int time_led=0;
long int time_pad=0; bool debounce_pad=false;
long int time_burn=0; bool debounce_burn=false;
long int time_launch=0;
long int time_flight=0;

//helper functions
void update_offsets();

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  Serial.begin(38400);
  Wire2.begin();

//  initialize Pressure
  if (!ps.init()) {
    while(1){
      Serial.println("Failed to autodetect pressure sensor!");
      digitalWrite(led, HIGH); delay(100);
      digitalWrite(led, LOW ); delay(100);
    };
  }
  ps.enableDefault();

//  initialize IMU
  if (imu.begin() != 0) {
    while(1){
      Serial.println("Failed to detect and initialize IMU!");
      digitalWrite(led, HIGH); delay(100);
      digitalWrite(led, LOW ); delay(100);
    };
  }
  imu.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  imu.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  imu.begin();
  
// initialize mag
  if (!mag.init()) {
    while(1){
      Serial.println("Failed to detect and initialize magnetometer!");
      digitalWrite(led, HIGH); delay(100);
      digitalWrite(led, LOW ); delay(100);
    };
  }
  mag.enableDefault();

// initialize servo
  fin1.attach(0); fin2.attach(1); fin3.attach(2); 
  fin1.writeMicroseconds(servo_min); fin2.writeMicroseconds(servo_min); fin3.writeMicroseconds(servo_max);
  delay(500);
  fin1.writeMicroseconds(servo_max); fin2.writeMicroseconds(servo_max); fin3.writeMicroseconds(servo_max);
  delay(500);
  fin1.writeMicroseconds(1500); fin2.writeMicroseconds(1500); fin3.writeMicroseconds(1500);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
      Serial.println("Card failed, or not present");
      digitalWrite(led, HIGH); delay(100);
      digitalWrite(led, LOW ); delay(100);
    }
  } else {
    int i=0;
    filename="BFR_log_"+String(i)+".txt";
    while (SD.exists(filename.c_str())) {
      i+=1;
      filename="BFR_log_"+String(i)+".txt";
    }
//     Serial.println("Card initialized, logging to: "+ filename);
  }
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);
  if (dataFile) {
    dataFile.println("t [ms], p [mbar], alt [m], T [c], ax [g], ay [g], az [g], gx [rad/s], gy [rad/s], gz [rad/s], q1, q2, q3, q4, state, u1, u2, u3");
    dataFile.close();
  } else {
   while (1){
    Serial.println("Error opening " + filename);
    digitalWrite(led, HIGH); delay(100);
    digitalWrite(led, LOW ); delay(100);
    }
  }

  // init timers
  time_0=millis(); time_p=millis();
}


void loop() {
  // // // // // // // // //
  // READ SENSORS
  // // // // // // // // //
  float pressure = ps.readPressureMillibars(); float altitude = ps.pressureToAltitudeMeters(pressure); float temperature = ps.readTemperatureC();
  ax_rv[i_rv]=imu.readFloatAccelX(); ay_rv[i_rv]=imu.readFloatAccelY(); az_rv[i_rv]=imu.readFloatAccelZ();
  // smooth out noisy gyro measurements
  gx_rv[i_rv]=imu.readFloatGyroX(); gy_rv[i_rv]=imu.readFloatGyroY(); gz_rv[i_rv]=imu.readFloatGyroZ(); i_rv=(i_rv+1)%s_rv;
  gx_r=0; gy_r=0; gz_r=0;
  
  // average across array
  float ax_r=0; float ay_r=0; float az_r=0;
  for(uint8_t i=0; i<s_rv; i++) {
    gx_r+=gx_rv[i]; gy_r+=gy_rv[i]; gz_r+=gz_rv[i];
    ax_r+=ax_rv[i]; ay_r+=ay_rv[i]; az_r+=az_rv[i];
  }
  gx_r/=s_rv; gy_r/=s_rv; gz_r/=s_rv;
  ax_r/=s_rv; ay_r/=s_rv; az_r/=s_rv;
  float ax=ax_r-ax_0; float ay=ay_r-ay_0; float az=az_r-az_0; 
  float gx=(gx_r-gx_0)*DEG_TO_RAD; float gy=(gy_r-gy_0)*DEG_TO_RAD; float gz=(gz_r-gz_0)*DEG_TO_RAD;

  mag.read(); float mx=mag.m.x; float my=mag.m.y; float mz=mag.m.z;

  // AHRS update
  float dt = ahrs.deltatUpdate(); time_p = millis();
  ahrs.MadgwickUpdate(gx,gy,gz,ax,ay,az,mx,my,mz,dt);
//  ahrs.MadgwickUpdate(gx,gy,gz,ax,ay,az,dt);
  float* q_r = ahrs.getQuat();
  quat_t q_imu = quat_t(q_r).norm();

  // // // // // // // // //
  // STATE TRANSITION LOGIC
  // // // // // // // // //
  if (millis()-time_0 > 10000) {
    state = FLIGHT; time_flight = millis();
  }
  
// IDLE TO PAD
  float a_norm = sqrt(ax_r*ax_r + ay_r*ay_r + az_r*az_r);
  if (state == IDLE && ax_r > 0.95 && a_norm > 0.95 && a_norm < 1.05) {
    if (debounce_pad) {
      if (millis()-time_pad > 5000){
        state = PAD; debounce_pad = false;
      }
    } else {debounce_pad = true; time_pad = millis();}
  } else {debounce_pad = false;}
  
//  EXIT PAD TO IDLE
  if (state == PAD && ax_r < 0.95 && a_norm > 0.95 && a_norm < 1.05) {
    state = IDLE;
  }

// PAD TO LAUNCH
  if (state == PAD && ax_r > 10) {
    if (debounce_burn) {
      if (millis()-time_burn > 250){
        state = LAUNCH; time_launch = millis(); debounce_burn = false;
      }
    } else {debounce_burn = true; time_burn = millis();}
  } else {debounce_burn = false;}

//  LAUNCH TO FLIGHT
  if (state == LAUNCH && millis()-time_launch>1400) {
    state = FLIGHT; time_flight = millis();
  }

  //  FLIGHT TO IDLE
  if (state == FLIGHT && millis()-time_flight>10000) {
    state = IDLE;
  }

  // // // // // // // // //
  // STATE IDLE
  // // // // // // // // //
  if (state == IDLE) {
//    light
    if (millis()-time_led>=1000) {digitalWrite(led, HIGH); time_led = millis();} 
    else {digitalWrite(led, LOW);}

    fin1.writeMicroseconds(1500); fin2.writeMicroseconds(1500); fin3.writeMicroseconds(1500);
  }

  // // // // // // // // //
  // STATE PAD
  // // // // // // // // //
  else if (state == PAD) {
    digitalWrite(led, HIGH);
    update_offsets();
    q_set = q_imu;

    fin1.writeMicroseconds(1500); fin2.writeMicroseconds(1500); fin3.writeMicroseconds(1500);
  }

  // // // // // // // // //
  // STATE LAUNCH
  // // // // // // // // //
  else if (state == LAUNCH) {

    fin1.writeMicroseconds(1500); fin2.writeMicroseconds(1500); fin3.writeMicroseconds(1500);
  }

  // // // // // // // // //
  // STATE FLIGHT
  // // // // // // // // //
  else if (state == FLIGHT) {
    //    light
    if (millis()-time_led>=200) {digitalWrite(led, HIGH); time_led = millis();} 
    else {digitalWrite(led, LOW);}
    
    quat_t q_diff = q_set * q_imu.conj();
    int8_t sgn = 1;
    if (q_diff.get(0) < 0) { sgn = -1; }
    
//    PD controller - diff coordinates based on Controls Scheme
    float Mbx = - sgn * Kp * q_diff.get(2) - Kd * gy;
    float Mby = - sgn * Kp * q_diff.get(3) - Kd * gz;
    float Mbz = - sgn * Kp * q_diff.get(1) - Kd * gx;
//    allocation
    Mbx *= Lx; Mby *= Ly; Mbz *= Lz;
    float sq3 = sqrt(3);
    L1 =        0 + 2*Mby + Mbz;
    L2 = -sq3*Mbx -   Mby + Mbz;
    L3 =  sq3*Mbx -   Mby + Mbz;
//    fin mapping
    u1 = map(L1,contr_min,contr_max,servo_min,servo_max);
    u2 = map(L2,contr_min,contr_max,servo_min,servo_max);
    u3 = map(L3,contr_min,contr_max,servo_min,servo_max);
//    limit
    u1 = min(servo_max,max(servo_min,u1));
    u2 = min(servo_max,max(servo_min,u2));
    u3 = min(servo_max,max(servo_min,u3));
//    servo output
    fin1.writeMicroseconds(u1); fin2.writeMicroseconds(u2); fin3.writeMicroseconds(u3);
  }

  // // // // // // // // //
  // LOG DATA
  // // // // // // // // //
  if (millis()-time_print > 100) { time_print = millis();
  // "t [ms], p [mbar], alt [m], T [c], ax [g], ay [g], az [g], gx [rad/s], gy [rad/s], gz [rad/s], q1, q2, q3, q4, state, u1, u2, u3"
    String dfs = String(millis())+", "+String(pressure)+", "+String(altitude)+", "+String(temperature)+", ";
    dfs += String(ax)+", "+String(ay)+", "+String(az)+", "+String(gx)+", "+String(gy)+", "+String(gz)+", ";
    dfs += String(q_imu.get(0))+", "+String(q_imu.get(1))+", "+String(q_imu.get(2))+", "+String(q_imu.get(3))+", ";
    dfs += String(state)+", "+String(u1)+", "+String(u2)+", "+String(u3);

  // open the file.
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dfs);
    dataFile.close();
  }

//    String ds = "ACC "+String(ax)+" "+String(ay)+" "+String(az)+" GYR "+String(gx)+" "+String(gy)+" "+String(gz)+" Q "+String(q0)+" "+String(q1)+" "+String(q2)+" "+String(q3);
//    String ds = String(gx)+" "+String(gy)+" "+String(gz);//+" "+String(gx_r)+" "+String(gy_r)+" "+String(gz_r)+" "+String(gx_0)+" "+String(gy_0)+" "+String(gz_0);
//    String ds = String(mag.m.x)+" "+String(mag.m.y)+" "+String(mag.m.z);
//    String ds = String(ax)+" "+String(ay)+" "+String(az);
//    String ds = String(gx_0)+" "+String(gy_0)+" "+String(gz_0);
//    String ds = String(ahrs.getRoll())+" "+String(ahrs.getPitch())+" "+String(ahrs.getYaw());
//    String ds = String(q_set.w())+" "+String(q_set.x())+" "+String(q_set.y())+" "+String(q_set.z());
//    String ds = String(q_imu.get(0))+" "+String(q_imu.get(1))+" "+String(q_imu.get(2))+" "+String(q_imu.get(3));
    String ds = String(u1)+" "+String(u2)+" "+String(u3);
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
