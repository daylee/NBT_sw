#define Encoder1A 2
#define Encoder1B 3
#define Encoder2A 4
#define Encoder2B 5
#define Encoder3A 6
#define Encoder3B 7
#define PWM_OUT_1 8
#define PWM_OUT_2 9
#define PWM_OUT_3 24

float RPM_1 = 0;
float RPM_2 = 0;
float RPM_3 = 0;

float Throttle_DS_1;

#include <PID_v1.h>


#include <Encoder.h>
#include <Chrono.h>
Chrono chrono_Encoder_1;
Chrono chrono_Encoder_2;
Chrono chrono_Encoder_3;
Chrono chrono_PID;
Encoder Encoder_1(Encoder1A,Encoder1B);
Encoder Encoder_2(Encoder2A,Encoder2B);
Encoder Encoder_3(Encoder3A,Encoder3B);
long oldPosition_1 = 0;
long oldPosition_2 = 0;
long oldPosition_3 = 0;
long newPosition_1 = 0;
long newPosition_2 = 0;
long newPosition_3 = 0;
int TimeLapse = 100;

float Commanded_RPM_1 = 0;
float Current_RPM_1 = 0;
float Commanded_RPM_2 = 0;
float Current_RPM_2 = 0;
float Commanded_RPM_3 = 0;
float Current_RPM_3 = 0;

#include <EasyTransfer.h>
#define HWSERIAL Serial1
EasyTransfer ET;

struct HW_Serial_Packet_Struct{
  float RW_A_RPM; // Commanded RPM for the reaction wheels
  float RW_B_RPM;
  float RW_C_RPM;
  // 214 bytes free for future use
  // This packet must be identical on the other side of the ET connection
};

HW_Serial_Packet_Struct HW_Serial_Packet;
double Setpoint,Input,Output;
PID myPID(&Input, &Output, &Setpoint,1,0,0,P_ON_M, DIRECT);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(100);
  ET.begin(details(HW_Serial_Packet),&Serial1);
  HWSERIAL.begin(9600);
  while(!Serial){
    delay(10);
  }
  delay(1000);
  Serial.println("Beginning Test");

  pinMode(Encoder1A,INPUT);
  pinMode(Encoder1B,INPUT);
  pinMode(Encoder2A,INPUT);
  pinMode(Encoder2B,INPUT);
  pinMode(Encoder3A,INPUT);
  pinMode(Encoder3B,INPUT);
  pinMode(PWM_OUT_1,OUTPUT);
  pinMode(PWM_OUT_2,OUTPUT);
  pinMode(PWM_OUT_3,OUTPUT);
  analogWriteFrequency(PWM_OUT_1,50);
  analogWriteFrequency(PWM_OUT_2,50);
  analogWriteFrequency(PWM_OUT_3,50);
  analogWriteResolution(15);

  myPID.SetMode(AUTOMATIC);

}

void loop() {
  RW_Position_Update();
  if(ET.receiveData()){
      Commanded_RPM_1 = HW_Serial_Packet.RW_A_RPM;
      Commanded_RPM_2 = HW_Serial_Packet.RW_B_RPM;
      Commanded_RPM_3 = HW_Serial_Packet.RW_C_RPM;
      Serial.print("RPM 1: ");
      Serial.println(Commanded_RPM_1);
      Serial.print("RPM 2: ");
      Serial.println(Commanded_RPM_2);
      Serial.print("RPM 3: ");
      Serial.println(Commanded_RPM_3);
    }
  else{
    delay(100);
    }
  RW_Position_Update();
  Velocity_Update_1();
  RW_Position_Update();
  Velocity_Update_2();
  RW_Position_Update();
  Velocity_Update_3();
  
  RW_PID_UPDATE();
  
  Serial.print(Velocity_Update_1());
  Serial.print(",");
  Serial.print(Velocity_Update_2());
  Serial.print(",");
  Serial.println(Velocity_Update_3());
 
  
//  Serial.print("Position 1: ");
//  Serial.println(newPosition_1);
//  Serial.print("Position 2: ");
//  Serial.println(newPosition_2);
//  Serial.print("Position 3: ");
//  Serial.println(newPosition_3);
  
}

void RW_Position_Update(){
  newPosition_1 = Encoder_1.read(); // Updates the encoder position for measuring the velocity of the reaction wheels, must be called as frequently as possible
  newPosition_2 = Encoder_2.read();
  newPosition_3 = Encoder_3.read();
}

void RW_PID_UPDATE(){
  int T = 25;
  int tol = 10;
  float Throttle_1;
  float Throttle_Offset_1;
  float error11;
  int error12;
  int error13;
  int last_error_1;
  float Throttle_DS_1;

  float Throttle_2;
  float Throttle_Offset_2;
  float error21;
  int error22;
  int error23;
  int last_error_2;
  float Throttle_DS_2;

  float Throttle_3;
  float Throttle_Offset_3;
  float error31;
  int error32;
  int error33;
  int last_error_3;
  float Throttle_DS_3;

  float Kp = 1.5;
  float Ki = 0.3;
  float Kd = 0.1;

  Throttle_1 = map(Commanded_RPM_1,-1500,1500,-1000,1000);
  Throttle_2 = map(Commanded_RPM_2,-1500,1500,-1000,1000);
  Throttle_3 = map(Commanded_RPM_3,-1500,1500,-1000,1000);

  if (Throttle_1 > 0){
    Throttle_1 = Throttle_1 + (Throttle_1*3 / 100);
  }
  if (Throttle_1 <0){
    Throttle_1 = Throttle_1 - (Throttle_1*3 / 100);
  }
  if (Throttle_2 > 0){
    Throttle_2 = Throttle_2 + (Throttle_2*3 / 100);
  }
  if (Throttle_2 <0){
    Throttle_2 = Throttle_2 - (Throttle_2*3 / 100);
  }
  if (Throttle_3 > 0){
    Throttle_3 = Throttle_3 + (Throttle_3*3 / 100);
  }
  if (Throttle_3 <0){
    Throttle_3 = Throttle_3 - (Throttle_3*3 / 100);
  }

  if(chrono_PID.elapsed()>T){
    chrono_PID.restart();
    Current_RPM_1 = Velocity_Update_1();
    error11 = Commanded_RPM_1-Current_RPM_1;
    Current_RPM_2 = Velocity_Update_2();
    error21 = Commanded_RPM_2-Current_RPM_2;
    Current_RPM_3 = Velocity_Update_3();
    error31 = Commanded_RPM_3-Current_RPM_3;

    if(abs(error11)<tol){
      error12 = 0;
    }
    else{
      Throttle_Offset_1 = (error11*Kp);
      error12 += error11;
      Throttle_Offset_1 += (error12*Ki);
      Throttle_Offset_1 += (error13*Kd);
      Throttle_Offset_1 = constrain(Throttle_Offset_1,-200,200);
      Throttle_1 = Throttle_1+Throttle_Offset_1;
    }
    if(abs(error21)<tol){
      error22 = 0;
    }
    else{
      Throttle_Offset_2 = (error21*Kp);
      error22 += error21;
      Throttle_Offset_2 += (error22*Ki);
      Throttle_Offset_2 += (error23*Kd);
      Throttle_Offset_2 = constrain(Throttle_Offset_2,-200,200);
      Throttle_2 = Throttle_2+Throttle_Offset_2;
    }
    if(abs(error31)<tol){
      error32 = 0;
    }
    else{
      Throttle_Offset_3 = (error31*Kp);
      error32 += error31;
      Throttle_Offset_3 += (error32*Ki);
      Throttle_Offset_3 += (error33*Kd);
      Throttle_Offset_3 = constrain(Throttle_Offset_3,-200,200);
      Throttle_3 = Throttle_3+Throttle_Offset_3;
    }

    error13 = last_error_1;
    error23 = last_error_2;
    error33 = last_error_3;

   Throttle_1 = constrain(Throttle_1,-1000,1000);
   Throttle_2 = constrain(Throttle_2,-1000,1000);
   Throttle_3 = constrain(Throttle_3,-1000,1000);
   Throttle_DS_1  = map(Throttle_1,-1000,1000,840,4076);
   Throttle_DS_2  = map(Throttle_2,-1000,1000,840,4076);
   Throttle_DS_3  = map(Throttle_3,-1000,1000,840,4076);
   analogWrite(PWM_OUT_1,Throttle_DS_1);
   analogWrite(PWM_OUT_2,Throttle_DS_2);
   analogWrite(PWM_OUT_3,Throttle_DS_3);
  
  }
}

float Velocity_Update_1(){ 
  if(chrono_Encoder_1.hasPassed(TimeLapse)){
    float Y = chrono_Encoder_1.elapsed();
    chrono_Encoder_1.restart();
    float X = newPosition_1 - oldPosition_1;
    RPM_1 = 60*(X*(1000/Y)/(28*4));
    oldPosition_1 = newPosition_1;
    return RPM_1;
  }
  else{
    return RPM_1;
  }
}

float Velocity_Update_2(){ 
  if(chrono_Encoder_2.hasPassed(TimeLapse)){
    float Y = chrono_Encoder_2.elapsed();
    chrono_Encoder_2.restart();
    float X = newPosition_2 - oldPosition_2;
    RPM_2 = 60*(X*(1000/Y)/(28*4));
    oldPosition_2 = newPosition_2;
    return RPM_2;
  }
  else{
    return RPM_2;
  }
}

float Velocity_Update_3(){ 
  if(chrono_Encoder_3.hasPassed(TimeLapse)){
    float Y = chrono_Encoder_3.elapsed();
    chrono_Encoder_3.restart();
    float X = newPosition_3 - oldPosition_3;
    RPM_3 = 60*(X*(1000/Y)/(28*4));
    oldPosition_3 = newPosition_3;
    return RPM_3;
  }
  else{
    return RPM_3;
  }
}
