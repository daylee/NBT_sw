// The big test script for the breadboard circuit

// This script written for Teensy 4.1 board

// By Gage Lochner

// Last edit 12/7/21

// Pin definitions
#define Encoder1A 2
#define Encoder1B 3
#define Encoder2A 4
#define Encoder2B 5
#define Encoder3A 6
#define Encoder3B 7
#define PWM_OUT_1 8
#define PWM_OUT_2 9
#define PWM_OUT_3 24
#define Dir1 22
#define Dir2 19
#define Dir3 16
#define Dir4 39
#define Step1 23
#define Step2 20
#define Step3 17
#define Step4 40
#define Enable1 21
#define Enable2 18
#define Enable3 41
#define Enable4 38
#define LS1 37
#define LS2 36
#define LS3 35
#define LS4 34
#define LSX 33


float RPM_1 = 0;
float RPM_2 = 0;
float RPM_3 = 0;


//General use variables
int Phase = 0; // The operational phase

//Library and setup for the reaction wheel encoders
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


// Library and setup for the stepper motor controllers
#include <AccelStepper.h>
AccelStepper CoMA1 = AccelStepper(1,Step1,Dir1);
AccelStepper CoMA2 = AccelStepper(1,Step2,Dir2);
AccelStepper CoMA3 = AccelStepper(1,Step3,Dir3);
AccelStepper CoMA4 = AccelStepper(1,Step4,Dir4);
int MaxSpeed = 6400*5;
int MaxAccel = 3200*5;
int MinPulseWidth = 40; // DO NOT CHANGE
float X1 = 0;
float X2 = 0;
float X3 = 0;
float X4 = 0;
bool First_Home = false;


// Library and setup for Hardware Serial 
#include <EasyTransfer.h>
#define HWSERIAL Serial1
EasyTransfer ET;

struct HW_Serial_Packet_Struct{
  // ET DATASTRUCT, received is commanded positions, sent is the current
  float CoMA1_X; // Position to move to in mm
  float CoMA2_X;
  float CoMA3_X;
  float CoMA4_X;
  int Operation_Phase; // To indicate what operation mode to operate in
  float RW_A_RPM; // Commanded RPM for the reaction wheels
  float RW_B_RPM;
  float RW_C_RPM;
  // 214 bytes free for future use
  // This packet must be identical on the other side of the ET connection
};

HW_Serial_Packet_Struct HW_Serial_Packet;


void setup() {
  // Serial setup, both for USB and for HWSERIAL to the embedded HUZZAH! board
  Serial.begin(9600);
  Serial.setTimeout(100);
  ET.begin(details(HW_Serial_Packet),&Serial1);
  HWSERIAL.begin(9600);
//  while(!Serial){ // Remove before actual use
//    delay(10);
//  }
//  delay(1000);
  Serial.println("Beginning Test");

  // Pin mode setup
  pinMode(Encoder1A,INPUT);
  pinMode(Encoder1B,INPUT);
  pinMode(Encoder2A,INPUT);
  pinMode(Encoder2B,INPUT);
  pinMode(Encoder3A,INPUT);
  pinMode(Encoder3B,INPUT);
  pinMode(PWM_OUT_1,OUTPUT);
  pinMode(PWM_OUT_2,OUTPUT);
  pinMode(PWM_OUT_3,OUTPUT);
  pinMode(Step1,OUTPUT);
  pinMode(Step2,OUTPUT);
  pinMode(Step3,OUTPUT);
  pinMode(Step4,OUTPUT);
  pinMode(Dir1,OUTPUT);
  pinMode(Dir2,OUTPUT);
  pinMode(Dir3,OUTPUT);
  pinMode(Dir4,OUTPUT);
  pinMode(Enable1,OUTPUT);
  pinMode(Enable2,OUTPUT);
  pinMode(Enable3,OUTPUT);
  pinMode(Enable4,OUTPUT);
  pinMode(LS1,INPUT_PULLDOWN); // Pull down to GND
  pinMode(LS2,INPUT_PULLDOWN);
  pinMode(LS3,INPUT_PULLDOWN);
  pinMode(LS4,INPUT_PULLDOWN);
  pinMode(LSX,INPUT_PULLDOWN); // Pull up to 3.3V

  analogWriteFrequency(PWM_OUT_1,50);
  analogWriteFrequency(PWM_OUT_2,50);
  analogWriteFrequency(PWM_OUT_3,50);
  analogWriteResolution(15);

  CoMA1.setMinPulseWidth(MinPulseWidth);
  CoMA2.setMinPulseWidth(MinPulseWidth);
  CoMA3.setMinPulseWidth(MinPulseWidth);
  CoMA4.setMinPulseWidth(MinPulseWidth);

  CoMA1.setMaxSpeed(MaxSpeed);
  CoMA2.setMaxSpeed(MaxSpeed);
  CoMA3.setMaxSpeed(MaxSpeed);
  CoMA4.setMaxSpeed(MaxSpeed);

  CoMA1.setAcceleration(MaxAccel);
  CoMA2.setAcceleration(MaxAccel);
  CoMA3.setAcceleration(MaxAccel);
  CoMA4.setAcceleration(MaxAccel);

  digitalWrite(Enable1,HIGH);
  digitalWrite(Enable2,HIGH);
  digitalWrite(Enable3,HIGH);
  digitalWrite(Enable4,HIGH);

}

void loop() {
  if(Phase == 0){
    if(ET.receiveData()){
      // Check the phase, and await for first message
      Phase = HW_Serial_Packet.Operation_Phase;
    }
    else{
      delay(50);
    }
  }

  if(Phase == 1){
    // Setup phase
    if(First_Home == false){
      CoMA_HOME();
      First_Home = true;
    }
    else{
      // Indicator LED 
    }
  }

  if(Phase == 2){
    // Automatic mass balancing phase
    // Future development
  }

  if(Phase == 3){
    // Manual control phase
    if(ET.receiveData()){
      CoMA_Position_Update();
      Commanded_RPM_1 = HW_Serial_Packet.RW_A_RPM;
      Commanded_RPM_2 = HW_Serial_Packet.RW_B_RPM;
      Commanded_RPM_3 = HW_Serial_Packet.RW_C_RPM;
      CoMA_Running_Update();
      RW_Position_Update();
      Velocity_Update_1();
      RW_Position_Update();
      Velocity_Update_2();
      RW_Position_Update();
      Velocity_Update_3();
      RW_PID_UPDATE();
    }
    else{
      CoMA_Running_Update();
      RW_Position_Update();
      Velocity_Update_1();
      RW_Position_Update();
      Velocity_Update_2();
      RW_Position_Update();
      Velocity_Update_3();
      RW_PID_UPDATE();
    }
  }
  if(Phase == 4){
    // Do nothing
  }

  if(Phase == 6){
    // E STOP PHASE
    SHUTDOWN();
  }

  if(Phase == 5){
    // Reserved for future mission scenarios
   
  }
  HW_Serial_Packet.CoMA1_X = CoMA1.currentPosition();
  HW_Serial_Packet.CoMA2_X = CoMA2.currentPosition();
  HW_Serial_Packet.CoMA3_X = CoMA3.currentPosition();
  HW_Serial_Packet.CoMA4_X = CoMA4.currentPosition();
  HW_Serial_Packet.RW_A_RPM = RPM_1;
  HW_Serial_Packet.RW_B_RPM = RPM_2;
  HW_Serial_Packet.RW_C_RPM = RPM_3;
  ET.sendData();
}

void CoMA_HOME(){
  // Call this function to home the stepper motor setups
  bool Homed1 = false;
  bool Homed2 = false;
  bool Homed3 = false;
  bool Homed4 = false;
  bool HomedAll = false;
  
  CoMA1.setSpeed(-3200*1.25);
  CoMA2.setSpeed(-3200*1.25);
  CoMA3.setSpeed(-3200*1.25);
  CoMA4.setSpeed(-3200*1.25);

  while (HomedAll == false){
    if(Homed1 == false){
      CoMA1.runSpeed();
      if(digitalRead(LS1) == HIGH){
        CoMA1.stop();
        CoMA1.setCurrentPosition(0);
        Homed1 = true;
      }
    }
    if(Homed2 == false){
      CoMA2.runSpeed();
      if(digitalRead(LS2) == HIGH){
        CoMA2.stop();
        CoMA2.setCurrentPosition(0);
        Homed2 = true;
      }
    }
    if(Homed3 == false){
      CoMA3.runSpeed();
      if(digitalRead(LS3) == HIGH){
        CoMA3.stop();
        CoMA3.setCurrentPosition(0);
        Homed3 = true;
      }
    }
    if(Homed4 == false){
      CoMA4.runSpeed();
      if(digitalRead(LS4) == HIGH){
        CoMA4.stop();
        CoMA4.setCurrentPosition(0);
        Homed4 = true;
      }
    }
    if(Homed1 == true && Homed2 == true && Homed3 == true && Homed4 == true){
      HomedAll = true;
    }
  }
  HomedAll = false;
  Homed1 = false;
  Homed2 = false;
  Homed3 = false;
  Homed4 = false;
  Serial.println("Initial home good on all 4 steppers");
  delay(500);
  CoMA1.runToNewPosition(3200);
  CoMA2.runToNewPosition(3200);
  CoMA3.runToNewPosition(3200);
  CoMA4.runToNewPosition(3200);
  delay(500);
  CoMA1.setSpeed(-3200/4);
  CoMA2.setSpeed(-3200/4);
  CoMA3.setSpeed(-3200/4);
  CoMA4.setSpeed(-3200/4);
  Serial.println("Going for second home");
  delay(500);  
  while (HomedAll == false){
    if(Homed1 == false){
      CoMA1.runSpeed();
      if(digitalRead(LS1) == HIGH){
        CoMA1.stop();
        CoMA1.setCurrentPosition(0);
        Homed1 = true;
      }
    }
    if(Homed2 == false){
      CoMA2.runSpeed();
      if(digitalRead(LS2) == HIGH){
        CoMA2.stop();
        CoMA2.setCurrentPosition(0);
        Homed2 = true;
      }
    }
    if(Homed3 == false){
      CoMA3.runSpeed();
      if(digitalRead(LS3) == HIGH){
        CoMA3.stop();
        CoMA3.setCurrentPosition(0);
        Homed3 = true;
      }
    }
    if(Homed4 == false){
      CoMA4.runSpeed();
      if(digitalRead(LS4) == HIGH){
        CoMA4.stop();
        CoMA4.setCurrentPosition(0);
        Homed4 = true;
      }
    }
    if(Homed1 == true && Homed2 == true && Homed3 == true && Homed4 == true){
      HomedAll = true;
    }
  }
  Serial.println("Final home good on all steppers");
  return;
}

void SHUTDOWN(){
  // Call this function to stop all motion in the system
  CoMA1.stop(); // Stop the stepper motors as quickly as possible
  CoMA2.stop();
  CoMA3.stop();
  CoMA4.stop();
  analogWrite(PWM_OUT_1,2457); // Send the idle signal to the RW motors
  analogWrite(PWM_OUT_2,2457);
  analogWrite(PWM_OUT_3,2457);
}



void CoMA_Position_Update(){
  // Call this function to update the position that the stepper motors will attempt to go to
  
  HW_Serial_Packet.CoMA1_X = constrain(HW_Serial_Packet.CoMA1_X,0,32*25.4);
  HW_Serial_Packet.CoMA2_X = constrain(HW_Serial_Packet.CoMA2_X,0,32*25.4);
  HW_Serial_Packet.CoMA3_X = constrain(HW_Serial_Packet.CoMA3_X,0,32*25.4);
  HW_Serial_Packet.CoMA4_X = constrain(HW_Serial_Packet.CoMA4_X,0,32*25.4);
  
  CoMA1.moveTo(HW_Serial_Packet.CoMA1_X*(6400/25.4));
  CoMA2.moveTo(HW_Serial_Packet.CoMA2_X*(6400/25.4));
  CoMA3.moveTo(HW_Serial_Packet.CoMA3_X*(6400/25.4));
  CoMA4.moveTo(HW_Serial_Packet.CoMA4_X*(6400/25.4));
  
}


void CoMA_Running_Update(){
  CoMA1.run(); // Updates the step and direction output of the stepper motor control pins, must be called as frequently as possible
  CoMA2.run();
  CoMA3.run();
  CoMA4.run();
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
