//project created by Ghaith Mhamdi
#include <SPI.h>           
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include "Wire.h" //I2C

int y=0,mini=1000,maxi=2000;  //maxi=2000;
Servo FR,FL,BR,BL;
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};
Data_Package data;
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;
unsigned long previousTime=0; //
unsigned long T=4000;

RF24 radio(8, 7);   // nRF24L01 (CE, CSN)

// PID
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;
//uint32_t LoopTimer;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};
/*
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;      //te3 lm3alem eli aalemni el PID
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;

float PRateRoll=0.35 ; float PRatePitch=PRateRoll; float PRateYaw=0.3;       // jeybhem mn doc chatgpt(5admet fi lco bl a3wej mnin mchet ldina)
float IRateRoll=0.08 ; float IRatePitch=IRateRoll; float IRateYaw=0.05;
float DRateRoll=0.008 ; float DRatePitch=DRateRoll; float DRateYaw=0.005;

float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=0.1 ; float IRatePitch=IRateRoll; float IRateYaw=1;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;

float PRateRoll=0.3 ; float PRatePitch=PRateRoll; float PRateYaw=2;         //lota stable ema ki aalitha fi lco kassretli edrone
float IRateRoll=1.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.011 ; float DRatePitch=DRateRoll; float DRateYaw=0;
*/
float PRateRoll=0.25 ; float PRatePitch=PRateRoll; float PRateYaw=2;      //fonctionne tres tres bien      
float IRateRoll=1.3 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.011 ; float DRatePitch=DRateRoll; float DRateYaw=0;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.000001*T/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/(0.000001*T);
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
}

void setup() {
  delay(2000);

  FR.attach(10);
  FL.attach(9);
  BL.attach(4);
  BR.attach(1);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening(); 
  resetData();

  while(!radio.available()); 
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); 
    while (data.pot2>5) {
      delay(4);
    }
  }
  //LoopTimer=micros();
  currentTime=micros();
}
void loop() {
  T=currentTime-previousTime;
  previousTime=currentTime;
  
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  //currentTime=millis();
  if (currentTime*0.001 - lastReceiveTime > 2000 ) {
    resetData();
    y-=(currentTime*0.001 - lastReceiveTime)*255/180000;   //tahbet fi essor3a elli tahbetha ki tkoun fi 255 fi 180ms=3minutes
    if(y<0){
      y=0;
    }
    data.pot2=y;
  }
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); 
    y=data.pot2;
    lastReceiveTime = millis(); 
  }

  InputThrottle=map(data.pot2, 0, 255, mini, maxi);

  //DesiredRatePitch   85-->115, 84-->114, 170-->140
  if (data.j1PotY>=140){
    DesiredRatePitch = map(data.j1PotY, 140, 255, 0, 30);    
  }
  else if(data.j1PotY<115){
    DesiredRatePitch =- map(data.j1PotY, 114, 0, 0, 30);
  }else{
    DesiredRatePitch=0;
  }
  
  //DesiredRateRoll
  if (data.j1PotX>=140){
    DesiredRateRoll =- map(data.j1PotX, 140, 255, 0, 30);       
  }
  else if(data.j1PotX<115){
    DesiredRateRoll = map(data.j1PotX, 114, 0, 0, 30);
  }else{
    DesiredRateRoll=0;
  }
  
  //DesiredRateYaw
  if (data.j2PotX >= 140) {
    DesiredRateYaw =- map(data.j2PotX, 140, 255, 0, 30);                              
  }
  else if (data.j2PotX < 115) {
    DesiredRateYaw = map(data.j2PotX, 114, 0, 0, 30);
  }else{
    DesiredRateYaw=0;
  }

  //ERRORS
  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  //PID
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll=PIDReturn[0];
  PrevErrorRateRoll=PIDReturn[1]; 
  PrevItermRateRoll=PIDReturn[2];
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch=PIDReturn[0]; 
  PrevErrorRatePitch=PIDReturn[1]; 
  PrevItermRatePitch=PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw,PrevItermRateYaw);
  InputYaw=PIDReturn[0]; 
  PrevErrorRateYaw=PIDReturn[1]; 
  PrevItermRateYaw=PIDReturn[2];

  InputThrottle=constrain(InputThrottle,mini,maxi-200);

  /*
  MotorInput1= 1.024*(InputThrottle-InputRoll+InputPitch+InputYaw);//FR
  MotorInput2= 1.024*(InputThrottle-InputRoll-InputPitch-InputYaw);//BR
  MotorInput3= 1.024*(InputThrottle+InputRoll-InputPitch+InputYaw);//BL
  MotorInput4= 1.024*(InputThrottle+InputRoll+InputPitch-InputYaw);//FL
  */
  
  MotorInput1= InputThrottle-InputRoll-InputPitch-InputYaw;//FR
  MotorInput2= InputThrottle-InputRoll+InputPitch+InputYaw;//BR
  MotorInput3= InputThrottle+InputRoll+InputPitch-InputYaw;//BL
  MotorInput4= InputThrottle+InputRoll-InputPitch+InputYaw;//FL
  

  MotorInput1=constrain(MotorInput1,mini,maxi);
  MotorInput2=constrain(MotorInput2,mini,maxi);
  MotorInput3=constrain(MotorInput3,mini,maxi);
  MotorInput4=constrain(MotorInput4,mini,maxi);

  if (data.pot2<=3) {
    MotorInput1=mini; 
    MotorInput2=mini;
    MotorInput3=mini; 
    MotorInput4=mini;
    reset_pid();
  }

  FR.writeMicroseconds(MotorInput1);
  FL.writeMicroseconds(MotorInput4);
  BL.writeMicroseconds(MotorInput3);                   
  BR.writeMicroseconds(MotorInput2);

  currentTime=micros();
}
void resetData(){
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 0;
  data.pot2 = 0;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;
}
