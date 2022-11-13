#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0x6B, 0x9A, 0xA0};
//94:B9:7E:6B:9A:A0



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

struct_message incomingIMUDAT;
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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingIMUDAT, incomingData, sizeof(incomingIMUDAT));
  Serial.print("Bytes received: ");
  Serial.println(len);
  AccelX = incomingIMUDAT.AX;
  AccelY = incomingIMUDAT.AY;
  AccelZ = incomingIMUDAT.AZ;
  GyroX = incomingIMUDAT.GX;
  GyroY = incomingIMUDAT.GY;
  GyroZ = incomingIMUDAT.GZ;
  MagX = incomingIMUDAT.MX;
  MagY = incomingIMUDAT.MY;
  MagZ = incomingIMUDAT.MZ;
  Serial.print("IMU AX: ");
  Serial.println(AccelX);
  Serial.print("IMU AY: ");
  Serial.println(AccelY);
  Serial.print("IMU AZ: ");
  Serial.println(AccelZ);
  Serial.print("IMU GX: ");
  Serial.println(GyroX);
  Serial.print("IMU GY: ");
  Serial.println(GyroY);
  Serial.print("IMU GZ: ");
  Serial.println(GyroZ);
  Serial.print("IMU MX: ");
  Serial.println(MagX);
  Serial.print("IMU MY: ");
  Serial.println(MagY);
  Serial.print("IMU MZ: ");
  Serial.println(MagZ);
  Serial.print("MODE : ");
  Serial.println(MODE);


}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(100);
  WiFi.mode(WIFI_STA);
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

  pinMode(21,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  getMODE();
  OUTGOING.MODE = MODE;
 // Send message via ESP-NOW
// Serial.println("i GO THERE");
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &OUTGOING, sizeof(OUTGOING));
   
  if (result == ESP_OK) {
//    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
//  Serial.println("I GO TTO EHER");
  if (MODE == 1){
    digitalWrite(21,HIGH);
          Serial.print("Mode isn't :");
      Serial.println(MODE);
  }
  else{
    digitalWrite(21,LOW);
          Serial.print("Mode isn't :");
      Serial.println(MODE);
  }
  delay(1000);\
}

void getMODE(){
  if(Serial.available()){
    while(Serial.available()){
      MODE = Serial.parseInt();
      Serial.print("Mode is :");
      Serial.println(MODE);
    }
  }
}
