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

// ESP NOW setup
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0x6B, 0xE7, 0x98};

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  // This runs automatically for each ESP NOW message received
  memcpy(&HW_Serial_Packet,incomingData,sizeof(HW_Serial_Packet_Struct)); // Copy received message to the data struct
  // The above line only works because the packets are currently identical
  // This may have to change in the future
  Serial.println("Data received");
  ET.sendData();
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(10);
  HWSERIAL.begin(9600);
  ET.begin(details(HW_Serial_Packet),&Serial1);

  while(!Serial){
    delay(10);
  }
  delay(1000);
  Serial.println("Beginning Test");

  // Wifi Setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { // Ensures that ESP NOW initializes
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv); // Sets up those automatic functions on data receive and send
  //esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo; // Sets up the peer info for the basestation board. Must be done for each board to be communicated with
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){ // Ensures peer add was sucsessful 
    Serial.println("Failed to add peer");
    return;
  }




}

void loop() {
  // put your main code here, to run repeatedly:
  
  
}
