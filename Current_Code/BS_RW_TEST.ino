#include <esp_now.h>
#include <WiFi.h>

// THIS IS THE SENDER SCRIPT

uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0x6B, 0x9A, 0xA0};

struct ESP_Packet_Struct{
  float RW_A_RPM; // Commanded RPM for the reaction wheels
  float RW_B_RPM;
  float RW_C_RPM;
  // 214 bytes free for future use
  // This packet must be identical on the other side of the ET connection
};

ESP_Packet_Struct ESP_Packet;




void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.setTimeout(10);

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
//  esp_now_register_recv_cb(OnDataRecv); // Sets up those automatic functions on data receive and send
//  esp_now_register_send_cb(OnDataSent);

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
if(Serial.available() > 0){
      while(Serial.available()>0 ){
        ESP_Packet.RW_A_RPM = Serial.parseInt();
        ESP_Packet.RW_B_RPM = Serial.parseInt();
        ESP_Packet.RW_C_RPM = Serial.parseInt();
        Serial.println("Data received");
        
        esp_err_t result = esp_now_send(0, (uint8_t *) &ESP_Packet, sizeof(ESP_Packet_Struct));
//        Serial.print("New throttle value sent");
//        Serial.println(Throttle_setting.Throttle_RW); 
      }
  }

}
