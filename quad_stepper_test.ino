#include<AccelStepper.h>

#define Dir1 22
#define Dir2 20
#define Dir3 18
#define Dir4 16
#define Step1 23
#define Step2 21
#define Step3 19
#define Step4 17
#define Enable1 41
#define Enable2 40
#define Enable3 39
#define Enable4 38
#define LS1 37
#define LS2 36
#define LS3 35
#define LS4 34
#define LSX 33
int MaxSpeed = 6400*5;
int MaxAccel = 3200*5;
int MinPulseWidth = 40;

float X1 = 0; // Position of the stepper in mms
float X2 = 0;
float X3 = 0;
float X4 = 0;

AccelStepper CoMA1 = AccelStepper(1,Step1,Dir1);
AccelStepper CoMA2 = AccelStepper(1,Step2,Dir2);
AccelStepper CoMA3 = AccelStepper(1,Step3,Dir3);
AccelStepper CoMA4 = AccelStepper(1,Step4,Dir4);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial){
    delay(10);
  }
  delay(1000);
  Serial.println("Beginning Test");
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
  pinMode(LS1,INPUT_PULLDOWN);
  pinMode(LS2,INPUT_PULLDOWN);
  pinMode(LS3,INPUT_PULLDOWN);
  pinMode(LS4,INPUT_PULLDOWN);
  pinMode(LSX,INPUT_PULLDOWN);

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

  HomeStep1();
  

}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(Enable1,HIGH);
  digitalWrite(Enable2,HIGH);
  digitalWrite(Enable3,HIGH);
  digitalWrite(Enable4,HIGH);
  if(Serial.available()>0){
     Serial_Parse();
  }
  CoMA1.run();
  CoMA2.run();
  CoMA3.run();
  CoMA4.run();
  
}

void HomeStep1(){
  
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

void Serial_Parse(){
  
  if(Serial.available() >0){
    while(Serial.available()>0){
      Serial.println("Reading serial data");
      X1 = Serial.parseInt();
      X2 = Serial.parseInt();
      X3 = Serial.parseInt();
      X4 = Serial.parseInt();
    }
  }
  
  X1 = constrain(X1,0,32*25.4); // CONSTRAIN THIS TO NEW LENGTH IN FORMAT (X*25.4)
  X2 = constrain(X2,0,32*25.4);
  X3 = constrain(X3,0,32*25.4);
  X4 = constrain(X4,0,32*25.4);
  Serial.println("Constraining the data");

  CoMA1.moveTo(X1*(6400/25.4));
  CoMA2.moveTo(X2*(6400/25.4));
  CoMA3.moveTo(X3*(6400/25.4));
  CoMA4.moveTo(X4*(6400/25.4));
  Serial.println("Setting the position of the stepper motors");
  Serial.print(X1);
  Serial.print(",");
  Serial.print(X2);
  Serial.print(",");
  Serial.print(X3);
  Serial.print(",");
  Serial.println(X4);

  
  
}
