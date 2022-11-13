#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0x6B, 0x45, 0x44};
//94:B9:7E:6B:45:44
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);

#include <EasyTransfer.h>
#define HWSERIAL Serial1
EasyTransfer ET;


struct HW_Serial_Packet_Struct{
  int MODE;
};


HW_Serial_Packet_Struct HW_Serial_Packet;

float AccelX;
float AccelY;
float AccelZ;
float GyroX;
float GyroY;
float GyroZ;
float MagX;
float MagY;
float MagZ;
int MODE;

typedef struct struct_message{
  float AX;
  float AY;
  float AZ;
  float GX;
  float GY;
  float GZ;
  float MX;
  float MY;
  float MZ;
  int MODE;
} struct_message;

struct_message INCOMING;
struct_message OUTGOING;

String success;

void OnDataSent(const uint8_t *mac_add,esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

esp_err_t result;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&INCOMING, incomingData, sizeof(INCOMING));
  Serial.print("Bytes received: ");
  Serial.println(len);
  MODE = INCOMING.MODE;

  if (MODE == 1){
    digitalWrite(21,HIGH);
  }
  else{
    digitalWrite(21,LOW);
  }

  result = esp_now_send(broadcastAddress, (uint8_t *) &OUTGOING, sizeof(OUTGOING));

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  pinMode(21,OUTPUT);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  if(!bno.begin()){
    Serial.println("No BNO sensor detected");
    while(1);    
  }
  delay(100);
  bno.setExtCrystalUse(true);// Using the crystal of the Microcontroller rather than the BNO crystal is more accurate
  HWSERIAL.begin(9600);
  ET.begin(details(HW_Serial_Packet),&Serial1);
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU_UPDATE();
 // Send message via ESP-NOW

 HW_Serial_Packet.MODE = MODE;
 Serial.print("HW :");
 Serial.print(HW_Serial_Packet.MODE);
 ET.sendData();
   
  if (result == ESP_OK) {
//    Serial.println("Sent with success");
    
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(1000);
}

void IMU_UPDATE(){
  
  imu::Vector<3> Gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  Serial.print("GX: ");
//  Serial.print(Gyro.x());
//  Serial.print(" GY: ");
//  Serial.print(Gyro.y());
//  Serial.print(" GZ: ");
//  Serial.print(Gyro.z());
//  Serial.println("");

  imu::Vector<3> Accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//  Serial.print("AX: ");
//  Serial.print(Accel.x());
//  Serial.print(" AY: ");
//  Serial.print(Accel.y());
//  Serial.print(" AZ: ");
//  Serial.print(Accel.z());
//  Serial.println("");

  imu::Vector<3> Mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
//  Serial.print("MX: ");
//  Serial.print(Mag.x());
//  Serial.print(" MY: ");
//  Serial.print(Mag.y());
//  Serial.print(" MZ: ");
//  Serial.print(Mag.z());
//  Serial.println("");
 
  imu::Quaternion quat = bno.getQuat();

//  /* Display the quat data */
//  Serial.print("qW: ");
//  Serial.print(quat.w(), 4);
//  Serial.print(" qX: ");
//  Serial.print(quat.y(), 4);
//  Serial.print(" qY: ");
//  Serial.print(quat.x(), 4);
//  Serial.print(" qZ: ");
//  Serial.print(quat.z(), 4);
//  Serial.println("");
  OUTGOING.AX = Accel.x();
  OUTGOING.AY = Accel.y();
  OUTGOING.AZ = Accel.z();
  OUTGOING.GX = Gyro.x();
  OUTGOING.GY = Gyro.y();
  OUTGOING.GZ = Gyro.z();
  OUTGOING.MX = Mag.x();
  OUTGOING.MY = Mag.y();
  OUTGOING.MZ = Mag.z();
  

}
