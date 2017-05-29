
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "MadgwickAHRS.h"
#include "PID_v1.h"
#include <Servo.h>
#include <SoftwareSerial.h>

/////////////////////MPU VARS
MPU6050 accelgyro;
Madgwick filter;
int16_t Rax, Ray, Raz;
int16_t Rgx, Rgy, Rgz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
////////////////////end mpu var


Servo servo1, servo2, servo3, servo4;
SoftwareSerial mySerial(3,2);

////////////////////////////////////////bluetooth VARS
int bluetoothTx = 12;
int bluetoothRx = 11;
SoftwareSerial bluetooth(bluetoothTx,bluetoothRx);
int throttle = 0;
////////////////////////////////////////end blue tooth var

char tempString[10];
char tempStringX[10];

int voltage=12,voltage2 =0;
bool IsBatteryConnect = true;

void setup() {
   Serial.begin(9600);    
   mySerial.begin(9600);  
   bluetooth.begin(9600);
   delay(500);
   mySerial.write(254); // move cursor to beginning of first line
   mySerial.write(128);
   mySerial.write("                                       ");
   mySerial.write(254);  
   mySerial.write(128);
   mySerial.write("Drone Te st     Connect  battery");
   while(IsBatteryConnect){
     printDroneData();
     delay(1000);
     Serial.println(voltage);
     if(voltage >11)
     {
       mySerial.write(254); // move cursor to beginning of first line
       mySerial.write(128);
       mySerial.write("                                       ");
       mySerial.write(254);  
       mySerial.write(128);
       mySerial.write("calibrat ing Esc:MAX PWM 179");
       servo1.attach(9);
       servo2.attach(5);
       servo1.write(179);
       servo2.write(179);
       delay(3000);
       servo1.write(0);
       servo2.write(0);
       mySerial.write(254); // move cursor to beginning of first line
       mySerial.write(128);
       mySerial.write("                                       ");
       mySerial.write(254); 
       mySerial.write(128);
       mySerial.write("calibrat ing Fin:MIN PWM 0");
       delay(3000);
       mySerial.write(254); // move cursor to beginning of first line
       mySerial.write(128);
       mySerial.write("                                       ");
       mySerial.write(254); 
       mySerial.write(128);
       mySerial.write("calibrat ing Fin:Start Motors");
       filter.begin(25);
       accelgyro.initialize();
       Serial.println("Testing device connections...");
       Serial.println(accelgyro.testConnection() 
       ? "MPU6050 connection successful" : "MPU6050 connection failed");
       Serial.print("Full Scale Gyro Range : ");
       Serial.print(accelgyro.getFullScaleGyroRange());
       Serial.print(" , Full Scale Accel Range : ");
       Serial.println(accelgyro. getFullScaleAccelRange());  
       IsBatteryConnect = false;
     }
   }

   
}

void loop() {
    if(bluetooth.available())
    {
           throttle = bluetooth.read();
           mySerial.write(254);  
           mySerial.write(128);
           mySerial.write("                                       ");    
           mySerial.write(254);  
           mySerial.write(128);
           mySerial.write("Speed te st "    );
           mySerial.write(254);  
           mySerial.write(192);
           mySerial.write(":PWM= ");
           sprintf(tempString,"%1d",throttle);
           mySerial.write(254);  
           mySerial.write(197);
           mySerial.write(tempString);



           
           servo1.attach(9);
           servo1.write(throttle);
           servo2.attach(5);
           servo2.write(throttle);
    }
    GetYawRollPitch();
    printDroneData();
     /*sprintf(tempStringX,"%1d",voltage);
     mySerial.write(254);  
     mySerial.write(205);
     mySerial.write("V");
     mySerial.write(254);  
     mySerial.write(206);
     mySerial.write(tempStringX);*/
}
////////////////////////////////////////////////////////////////////////////////////////////////////MPU FUNTIONS START
void GetYawRollPitch()
{
   accelgyro.getMotion6(&Rax, &Ray, &Raz, &Rgx, &Rgy, &Rgz);
  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(Rax);
  ay = convertRawAcceleration(Ray);
  az = convertRawAcceleration(Raz);
  gx = convertRawGyro(Rgx);
  gy = convertRawGyro(Rgy);
  gz = convertRawGyro(Rgz);

  filter.updateIMU(gx, gy, gz, ax, ay, az);
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
}
float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  float g = (gRaw * 250.0) / 32768 ;
  return g;
}
float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 3276 
  float a = (aRaw * 2.0) / 32768 ;
  return a;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////MPU FUNTIONS END

void printDroneData()
{
      Serial.println();
      Serial.print("Roll= ");
      Serial.print(roll);
      Serial.print(" |Pitch= ");
      Serial.print(pitch);
      Serial.print(" |YAW= ");
      Serial.print(heading);
      int sensorValue = analogRead(A0);
      voltage = (sensorValue * (4.3 / 1023.0))*(12/4.3);
      //int sensorValue2 = analogRead(A0);
      //voltage2 = (sensorValue2 * (4.3 / 1023.0))*(12/4.3);
      Serial.print("| voltage 1 and 2 : One ");
      Serial.print(voltage);
      Serial.print(" | two = ");
      Serial.print(voltage2);
}
 
