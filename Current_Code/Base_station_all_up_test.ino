 // Base station Huzzah board component for all up test script

// By Gage Lochner

// Finalized on 12/6/2021

// LCD setup and library
#include <LiquidCrystal.h>
const int rs = 15, en = 33, d4 = 32, d5 = 14, d6 = 22, d7 = 23;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Heartbeat timer
#include <Chrono.h>
Chrono chrono_HB;

// The library for the buzzer
#include <Tone32.h>

// Button setup
#define B1B 5 // Button 1 pin
// NOTE: Apparently "B1" is used by something in the ESP32 core. Adding an extra letter to the variable fixed the issue. 
#define B2 18 // Button 2 pin
#define B3 19 // Button 3 pin
#define B4 16 // Button 4 pin
#define B5 17 // Button 5 pin
#define B6 21 // Button 5 pin
bool ARMED = false; // false if system is not armed, true once the system is armed
int Prior = 0; // Phase found in the last loop
int phase = 0; // Current phase / most up to date known phase

// Debounce library setup
#include <Bounce2.h>

Bounce Button1 = Bounce();
Bounce Button2 = Bounce();
Bounce Button3 = Bounce();
Bounce Button4 = Bounce();
Bounce Button5 = Bounce();
Bounce Button6 = Bounce();

// ESP NOW
#include <esp_now.h>
#include<WiFi.h>

uint8_t broadcastAddress[] = {0xC4,0xDD,0x57,0x9C,0xC8,0x64}; // MAC address to send data to
// MAC address of the embedded board

void OnDataSent(const uint8_t *mac_addr,esp_now_send_status_t status){
  // This runs automatically for each ESP now message sent
  Serial.println("ESP SENT");
  
}


struct ESP_DATA_SEND_Struct{
  float CoMA1;
  float CoMA2;
  float CoMA3;
  float CoMA4;
  int Phase;
  float RW_RPM1;
  float RW_RPM2;
  float RW_RPM3;
};

ESP_DATA_SEND_Struct ESP_DATA_SEND;

struct ESP_DATA_RECV_Struct{
  float Acc_X;
  float Acc_Y;
  float Acc_Z;
  float Gyro_X;
  float Gyro_Y;
  float Gyro_Z;
  float Mag_X;
  float Mag_Y;
  float Mag_Z;
  float Quat_W;
  float Quat_X;
  float Quat_Y;
  float Quat_Z;
  float CoMA1;
  float CoMA2;
  float CoMA3;
  float CoMA4;
  int Phase;
  float RW_RPM1;
  float RW_RPM2;
  float RW_RPM3;
  float Batt_Voltage;
  float Time;
};

ESP_DATA_RECV_Struct ESP_DATA_RECV;

#define buzzer 16


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  // This runs automatically for each ESP NOW message received
  memcpy(&ESP_DATA_RECV,incomingData,sizeof(ESP_DATA_RECV_Struct));
  chrono_HB.restart(); // Reset the heartbeat timer since we heard from the embedded system
  Serial.print(ESP_DATA_RECV.Acc_X); // Print out the data received from the embedded system
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Acc_Y);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Acc_Z);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Gyro_X);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Gyro_Y);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Gyro_Z);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Mag_X);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Mag_Y);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Mag_Z);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Quat_W);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Quat_X);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Quat_Y);
  Serial.print(",");
  Serial.print(ESP_DATA_RECV.Quat_Z); 
  Serial.print(",");
  Serial.println(ESP_DATA_RECV.Batt_Voltage);

  
}


void setup() {
  Serial.begin(9600);
  
  Button1.attach(B1B,INPUT); // Setup bedounce for each button
  Button2.attach(B2,INPUT);
  Button3.attach(B3,INPUT);
  Button4.attach(B4,INPUT);
  Button5.attach(B5,INPUT);
  Button6.attach(B6,INPUT);
  Button1.interval(10); // 10 ms debounce interval
  Button2.interval(10);
  Button3.interval(10);
  Button4.interval(10);
  Button5.interval(10);
  Button6.interval(10);
  pinMode(rs,OUTPUT); // Pin mode setup for the LCD
  pinMode(en,OUTPUT);
  pinMode(d4,OUTPUT);
  pinMode(d5,OUTPUT);
  pinMode(d6,OUTPUT);
  pinMode(d7,OUTPUT);
  pinMode(buzzer,OUTPUT); // Pin mode setup for the buzzer
  lcd.begin(16, 2); // Begin the LCD object
  lcd.print("Status: Unarmed"); // print to lcd
  lcd.setCursor(0,1); // move the cursor to the second line of the LCD

  while(ARMED == false){ // While system is unarmed
    phase = button_update(); // Check to see if a button has been pressed
    if (ARMED == false){ // If no button has been pressed
      lcd.setCursor(0,1);
      lcd.print("Waiting to arm");
      delay(10);
    }
    if(ARMED == true){ // If a button was pressed
      lcd.clear(); // Clear the LCD of any text
      lcd.setCursor(0,0); // Print out a message with a build in time delay while the system is "Arming"
      lcd.print("Status: ARMIMG");
      lcd.setCursor(0,1);
      lcd.print("3");
      delay(250);
      lcd.print(".");
      delay(250);
      lcd.print(".");
      delay(250);
      lcd.print(".");
      delay(250);
      lcd.print(" 2");
      delay(250);
      lcd.print(".");
      delay(250);
      lcd.print(".");
      delay(250);
      lcd.print(".");
      delay(250);
      lcd.print(" 1");
      delay(750);
      lcd.print("ARMED");
      phase = 0;
    }
  }
  lcd.clear();

  phase = 0;

  // Wifi Setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { // Ensures that ESP NOW initializes
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv); // Sets up those automatic functions on data receive and send
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo; // Sets up the peer info for the basestation board. Must be done for each board to be communicated with
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){ // Ensures peer add was sucsessful 
    Serial.println("Failed to add peer");
    return;
  }
  while (phase == 0){ // If the phase 1 button wasn't pressed, the system will wait for the phase 1 button to be pressed, since we will always want to start in phase 1
    lcd.setCursor(0,0);
    lcd.print("Press Phase 1");
    lcd.setCursor(0,1);
    lcd.print("To start");
    Button1.update();
    Button2.update();
    Button3.update();
    Button4.update();
    Button5.update();
    Button6.update();
    if(Button1.read() == LOW) {
      // Do nothing
    }
    else{
      phase = button_update();
      lcd.clear();
      lcd.print("Status: Armed");
    }
  }
  phase = button_update();
  lcd.setCursor(0,1);
  lcd.print("Phase:");
  lcd.print(phase);

  chrono_HB.start(); // Start the heartbeat timer

}

void loop() {
  phase = button_update(); // Check for a button update
  if(HB_CHECK() == true){ // If the HB check fails
     HB_CHECK(); // Check again
     SOUND_THE_BUZZER(); // sound the buzzer
     return; // And return
  }
  else if(Low_Batt_Alarm() == true){ // If the battery of the embedded system is low
    SOUND_THE_BUZZER(); // sound the buzzer
    return; // And return
  }

  else if(phase == 1){
    // Do phase 1 things
    ESP_DATA_SEND.Phase = phase;
    lcd.setCursor(0,0);
    lcd.print("Status:Armed      ");
    lcd.setCursor(0,1);
    lcd.print("Phase 1         ");
  }
  else if(phase == 2){
    // Do phase 2 things
    ESP_DATA_SEND.Phase = phase;
    lcd.setCursor(0,0);
    lcd.print("Status:Armed      ");
    lcd.setCursor(0,1);
    lcd.print("Phase 2         ");
  }
  else if(phase == 3){
    // Do phase 3 things
    ESP_DATA_SEND.Phase = phase;
    lcd.setCursor(0,0);
    lcd.print("Status:Armed      ");
    lcd.setCursor(0,1);
    lcd.print("Phase 3         ");
  }
  else if(phase == 4){
    // Do phase 4 things
    ESP_DATA_SEND.Phase = phase;
    lcd.setCursor(0,0);
    lcd.print("Status:Armed      ");
    lcd.setCursor(0,1);
    lcd.print("Phase 4           ");
  }
  else if(phase == 5){
    // Do phase 5 things
    ESP_DATA_SEND.Phase = phase;
    lcd.setCursor(0,0);
    lcd.print("Status:Armed      ");
    lcd.setCursor(0,1);
    lcd.print("Phase 5           ");
  }
  else if(phase == 6){
    // Do phase 6 things
    ESP_DATA_SEND.Phase = phase;
    lcd.setCursor(0,0);
    lcd.print("Status:Armed      ");
    lcd.setCursor(0,1);
    lcd.print("Phase 6           ");
    
  }
  Serial_Read_From_Matlab();
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ESP_DATA_SEND, sizeof(ESP_DATA_SEND_Struct));

}

int button_update(){
  Button1.update();
  Button2.update();
  Button3.update();
  Button4.update();
  Button5.update();
  Button6.update();
  
  if(Button1.read() == HIGH){
    ARMED = true;
    Prior = 1;
    return 1;
  }
  else if(Button2.read() == HIGH){
    ARMED = true;
    Prior = 2;
    return 2;
  }
  else if(Button3.read() == HIGH){
    ARMED = true;
    Prior = 3;
    return 3;
  }
  else if(Button4.read() == HIGH){
    ARMED = true;
    Prior = 4;
    return 4;
  }
  else if(Button5.read() == HIGH){
    ARMED = true;
    Prior = 5;
    return 5;
  }
  else if(Button6.read() == HIGH){
    ARMED = true;
    Prior = 6;
    return 6;
  }
  else{
    return Prior; 
  }
}

void Serial_Read_From_Matlab(){
  if(Serial.available()){
    while(Serial.available()){
      // Read in the variables into the struct to be sent
      ESP_DATA_SEND.CoMA1 = Serial.parseInt();
      ESP_DATA_SEND.CoMA2 = Serial.parseInt();
      ESP_DATA_SEND.CoMA3 = Serial.parseInt();
      ESP_DATA_SEND.CoMA4 = Serial.parseInt();
      ESP_DATA_SEND.RW_RPM1 = Serial.parseInt();
      ESP_DATA_SEND.RW_RPM2 = Serial.parseInt();
      ESP_DATA_SEND.RW_RPM3 = Serial.parseInt();
      Serial.println("data read in");
    }
  }
}

bool HB_CHECK(){
  if(chrono_HB.hasPassed(10000)){
    Serial.println("NO HEARTBEAT");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("NO HEARTBEAT   ");
    lcd.setCursor(0,1);
    lcd.print("for ");
    lcd.print(chrono_HB.elapsed()/1000);
    lcd.print("s");
    return true;
  }
  else{
    return false;
  }
  
}

bool Low_Batt_Alarm(){
  if(ESP_DATA_RECV.Batt_Voltage > 19.5){ // Toggle this comparison 
    SOUND_THE_BUZZER();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("LOW BATTERY   ");
    lcd.setCursor(0,1);
    lcd.print("SYSTEM SHUTDOWN");
    return true;
  }
  else{
    return false;
  }
}

void SOUND_THE_BUZZER(){
  tone(12, NOTE_G6, 333, 0);
}
