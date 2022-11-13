#include <EasyTransfer.h>
#define HWSERIAL Serial1
EasyTransfer ET;

int MODE;

struct HW_Serial_Packet_Struct{
  int MODE;
};


HW_Serial_Packet_Struct HW_Serial_Packet;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ET.begin(details(HW_Serial_Packet),&Serial1);
  HWSERIAL.begin(9600);
  pinMode(32,OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(ET.receiveData()){
    MODE = HW_Serial_Packet.MODE;
    Serial.println("ET");
    Serial.println(MODE);
    Serial.println(HW_Serial_Packet.MODE);
  }
  if(MODE == 1){
    digitalWrite(32,HIGH);
  }
  else{
    digitalWrite(32,LOW);
  }
}
