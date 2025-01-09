#include <Wire.h>
//for the d1 mini

int rmotor1 = D6; // GPIO12
int rmotor2 = D5; // GPIO13 
int lmotor1 = D8; // GPIO14
int lmotor2 = D7; // GPIO15
int mspeed = 10;
int turnspeed=50;

//
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float elapsedTime, curTime, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

//

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;



float kp=25;
float ki=0;
float kd=0.8;
float desired_angle = 0;

void setup() 
{
  Wire.begin(D2, D1); // Initialize I²C (SDA = D2, SCL = D1)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  // Serial.print("HEY!");


  pinMode(rmotor1,OUTPUT);
  //pinMode(lmotor1,OUTPUT);
  pinMode(rmotor2,OUTPUT);
  //pinMode(lmotor2,OUTPUT);
  curTime = millis();


}
void loop() 
{
   timePrev = curTime;  
    curTime = millis();  
    elapsedTime = (curTime - timePrev) / 1000; 
    
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);
    //Read IMU data
    Acc_rawX=Wire.read()<<8|Wire.read(); 
    Acc_rawY=Wire.read()<<8|Wire.read();
    Acc_rawZ=Wire.read()<<8|Wire.read(); 
    //find angles
    Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    Wire.beginTransmission(0x68);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true); 
    //read gyro
    Gyr_rawX=Wire.read()<<8|Wire.read(); 
    Gyr_rawY=Wire.read()<<8|Wire.read(); 
    //find angles
    Gyro_angle[0] = Gyr_rawX/131.0; 
    Gyro_angle[1] = Gyr_rawY/131.0;
    Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];//complementary filter
    Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
    //Total_angle[1] =  83.8;
    float temp = Total_angle[0];
    Total_angle[0] = Total_angle[1];
    Total_angle[1] = temp;

    Total_angle[0] += 60;
    // Serial.print(Total_angle[0]);
    // Serial.print("<-----pitch               roll-------->");
    // Serial.print(Total_angle[1]);
    // Serial.print("            ");

    
    error = Total_angle[0] - desired_angle;
    pid_p = kp*error;//proportional
    pid_i = pid_i+(ki*error);//integral
    pid_d = kd*((error - previous_error)/elapsedTime);//derivative
    PID = pid_p + pid_d;
    previous_error = error;
   
    mspeed = abs(PID);
    if(mspeed > 1023) mspeed = 1023;
    
   
   if(Total_angle[0]<0)
      {
       anti();
      }
    if(Total_angle[0]>0)
      {
       clockw();
      }
    if(Total_angle[0]>45)
     halt();
    if(Total_angle[0]<-45)
      halt();
    Total_angle[0]-=60;
    
  
}
void clockw()
{
  analogWrite(rmotor1,mspeed);
  analogWrite(rmotor2,0);
  //analogWrite(lmotor1,mspeed);
  //analogWrite(lmotor2,0); 
}
void anti()
{

  analogWrite(rmotor2,mspeed);
  analogWrite(rmotor1,0);
  //analogWrite(lmotor2,mspeed);
  //analogWrite(lmotor1,0);
}
void halt()
{
  
  analogWrite(rmotor1,0);
  analogWrite(rmotor2,0);
  //analogWrite(lmotor1,0);
  //analogWrite(lmotor2,0);
  
}