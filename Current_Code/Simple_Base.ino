// The script below is the simplest code that can accomplish all needed goals for a tabletop test

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0x6B, 0x9A, 0xA0};
//Change to the MAC address of the embedded board being used

#include <Chrono.h>
Chrono chrono_PO;

float AccelX;
float AccelY;
float AccelZ;
float GyroX;
float GyroY;
float GyroZ;
float MagX;
float MagY;
float MagZ;
float QuatW;
float QuatX;
float QuatY;
float QuatZ;

typedef struct struct_message_IN{
  float AX; // IMU data
  float AY;
  float AZ;
  float GX;
  float GY;
  float GZ;
  float MX;
  float MY;
  float MZ;
  float QW;
  float QX;
  float QY;
  float QZ;
} struct_message_IN;


typedef struct struct_message_OUT{
  float CoMA1_X; // Position to move to in mm
  float CoMA2_X;
  float CoMA3_X;
  float CoMA4_X;
  float RW_A_RPM; // Commanded RPM for the reaction wheels
  float RW_B_RPM;
  float RW_C_RPM;
} struct_message_OUT;

struct_message_IN INCOMING;
struct_message_OUT OUTGOING;

String success;

void OnDataSent(const uint8_t *mac_add,esp_now_send_status_t status){
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
//    success = "Delivery Success :)";
  }
  else{
//    success = "Delivery Fail :(";
  }
  // Turn the above off once it's working correctly. The basestation doesn't need to do anything on data send
  // Alternatively, print it to the LCD screen
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&INCOMING, incomingData, sizeof(INCOMING));
  AccelX = INCOMING.AX;
  AccelY = INCOMING.AY;
  AccelZ = INCOMING.AZ;
  GyroX = INCOMING.GX;
  GyroY = INCOMING.GY;
  GyroZ = INCOMING.GZ;
  MagX = INCOMING.MX;
  MagY = INCOMING.MY;
  MagZ = INCOMING.MZ;
  QuatW = INCOMING.QW;
  QuatX = INCOMING.QX;
  QuatY = INCOMING.QY;
  QuatZ = INCOMING.QZ;

  if(chrono_PO.hasPassed(1000)){
  // USE EITHER OF THE BELOW FUNCTIONS
    HUMAN_READ_PRINTOUT();
//    MATLAB_PRINTOUT();
    chrono_PO.restart();
  }
}

void setup(){
  Serial.begin(9600);
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
  // Register for a callback function that will be called when data is received or sent
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

}

void loop(){
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &OUTGOING, sizeof(struct_message_OUT)); // Send the ESP now command packet
  if (result == ESP_OK) {
//    Serial.println("Sent with success - main loop message");
  }
  else {
//    Serial.println("Error sending the data - main loop message");
  }
  getCommands();
}

void getCommands(){
  if(Serial.available()){
    while(Serial.available()){
      OUTGOING.CoMA1_X = Serial.parseInt();
      OUTGOING.CoMA2_X = Serial.parseInt();
      OUTGOING.CoMA3_X = Serial.parseInt();
      OUTGOING.CoMA4_X = Serial.parseInt();
      OUTGOING.RW_A_RPM = Serial.parseInt();
      OUTGOING.RW_B_RPM = Serial.parseInt();
      OUTGOING.RW_C_RPM = Serial.parseInt();
      Serial.println("New set of commands read in correctly");
      Serial.print(OUTGOING.CoMA1_X);
      Serial.print(",");
      Serial.print(OUTGOING.CoMA2_X);
      Serial.print(",");
      Serial.print(OUTGOING.CoMA3_X);
      Serial.print(",");
      Serial.print(OUTGOING.CoMA4_X);
      Serial.print(",");
      Serial.print(OUTGOING.RW_A_RPM);
      Serial.print(",");
      Serial.print(OUTGOING.RW_B_RPM);
      Serial.print(",");
      Serial.println(OUTGOING.RW_C_RPM);
    }
  }
}

void HUMAN_READ_PRINTOUT(){
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
  Serial.print("IMU QW: ");
  Serial.println(QuatW);
  Serial.print("IMU QX: ");
  Serial.println(QuatX);
  Serial.print("IMU QY: ");
  Serial.println(QuatY);
  Serial.print("IMU QZ: ");
  Serial.println(QuatZ);
}

void MATLAB_PRINTOUT(){
  Serial.print(AccelX);
  Serial.print(",");
  Serial.print(AccelY);
  Serial.print(",");
  Serial.print(AccelZ);
  Serial.print(",");
  Serial.print(GyroX);
  Serial.print(",");
  Serial.print(GyroY);
  Serial.print(",");
  Serial.print(GyroZ);
  Serial.print(",");
  Serial.print(MagX);
  Serial.print(",");
  Serial.print(MagY);
  Serial.print(",");
  Serial.print(MagZ);
  Serial.print(",");
  Serial.print(QuatW);
  Serial.print(",");
  Serial.print(QuatX);
  Serial.print(",");
  Serial.print(QuatY);
  Serial.print(",");
  Serial.println(QuatZ);
}
