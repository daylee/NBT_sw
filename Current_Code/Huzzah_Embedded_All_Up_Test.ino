// The big test script for the breadboard circuit

// This script written for Huzzah! board

// By Gage Lochner

// Last edit 12/7/21

// Pin definitions
#define Water_Sensor A2
#define Signal_LED_Data 21
#define Battery_Monitor A3

float R1 = 15*1000;
float R2 = 1*1000;


// IMU library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// Define the sample rate and the I2C address of the sensor (can be adjusted for using more than 1 at once)
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);

// Indicator LEDs
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip(1,Signal_LED_Data); // Change to indicate the number of LEDS used

// Library and setup for Hardware Serial 
#include <EasyTransfer.h>
#define HWSERIAL Serial1
EasyTransfer ET;

struct HW_Serial_Packet_Struct{
  // ET DATASTRUCT, received is current positions, sent is the commanded
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

struct ESP_DATA_RECV{
  float CoMA1;
  float CoMA2;
  float CoMA3;
  float CoMA4;
  int Phase;
  float RW_RPM1;
  float RW_RPM2;
  float RW_RPM3;
};

ESP_DATA_RECV ESP_DATA_RECV_PACKET;

struct ESP_DATA_SEND{
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

ESP_DATA_SEND ESP_DATA_SEND_PACKET;

// ESP NOW setup
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x94,0xB9,0x7E,0x6B,0xE7,0x98};
// MAC address of the basestation 

void OnDataSent(const uint8_t *mac_addr,esp_now_send_status_t status){
  // code go here
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  // This runs automatically for each ESP NOW message received
  HW_Serial_Packet.CoMA1_X = ESP_DATA_RECV_PACKET.CoMA1;
  HW_Serial_Packet.CoMA2_X = ESP_DATA_RECV_PACKET.CoMA2;
  HW_Serial_Packet.CoMA3_X = ESP_DATA_RECV_PACKET.CoMA3;
  HW_Serial_Packet.CoMA4_X = ESP_DATA_RECV_PACKET.CoMA4;
  HW_Serial_Packet.Operation_Phase = ESP_DATA_RECV_PACKET.Phase;
  HW_Serial_Packet.RW_A_RPM = ESP_DATA_RECV_PACKET.RW_RPM1;
  HW_Serial_Packet.RW_B_RPM = ESP_DATA_RECV_PACKET.RW_RPM2;
  HW_Serial_Packet.RW_C_RPM = ESP_DATA_RECV_PACKET.RW_RPM3;
  
  
  
}

void setup() {
  // Serial setup, both for USB and for HWSERIAL to the embedded HUZZAH! board
//  Serial.begin(115200);
//  Serial.setTimeout(100);
  ET.begin(details(HW_Serial_Packet),&Serial1); // Sets up ET
  HWSERIAL.begin(9600);
//  while(!Serial){
//    delay(10);
//  }
//  delay(1000);
//  Serial.println("Beginning Test");

  // IMU SETUP
  if(!bno.begin()){
//    Serial.println("No BNO sensor detected");
    while(1);    
  }
  delay(100);
  bno.setExtCrystalUse(true);// Using the crystal of the Microcontroller rather than the BNO crystal is more accurate

  // Wifi Setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { // Ensures that ESP NOW initializes
//    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv); // Sets up those automatic functions on data receive and send
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo; // Sets up the peer info for the basestation board. Must be done for each board to be communicated with
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
         
  if (esp_now_add_peer(&peerInfo) != ESP_OK){ // Ensures peer add was sucsessful 
//    Serial.println("Failed to add peer");
    return;
  }
  
  // LED setup
  strip.begin();
  strip.show();
  strip.setPixelColor(0,0,0,0);
  strip.show();

  // Pinmodes
  pinMode(Water_Sensor,INPUT);
  pinMode(Signal_LED_Data,OUTPUT);
  pinMode(Battery_Monitor,INPUT);

}

bool Calibration_status = true;


void loop() {
  IMU_UPDATE();
  Water_Sensor_Update();
  Battery_Check();
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ESP_DATA_SEND_PACKET, sizeof(ESP_DATA_SEND));
  ET.sendData();
  if(ET.receiveData()){
    ESP_DATA_SEND_PACKET.CoMA1 = HW_Serial_Packet.CoMA1_X;
    ESP_DATA_SEND_PACKET.CoMA2 = HW_Serial_Packet.CoMA2_X;
    ESP_DATA_SEND_PACKET.CoMA3 = HW_Serial_Packet.CoMA3_X;
    ESP_DATA_SEND_PACKET.CoMA4 = HW_Serial_Packet.CoMA4_X;
    ESP_DATA_SEND_PACKET.RW_RPM1 = HW_Serial_Packet.RW_A_RPM;
    ESP_DATA_SEND_PACKET.RW_RPM2 = HW_Serial_Packet.RW_B_RPM;
    ESP_DATA_SEND_PACKET.RW_RPM3 = HW_Serial_Packet.RW_C_RPM;
  }
}

void IMU_Calibration(){
 while(Calibration_status == false){
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
//    Serial.println();
//    Serial.print("Calibration: Sys=");
//    Serial.print(system);
//    Serial.print(" Gyro=");
//    Serial.print(gyro);
//    Serial.print(" Accel=");
//    Serial.print(accel);
//    Serial.print(" Mag=");
//    Serial.println(mag);
  
//    Serial.println("--");
    delay(100);
    if(gyro == 3 && mag == 3 && system == 3 && accel > 2){
      Calibration_status = true;
//      Serial.println("You are now calibrated");
      delay(1000);
//      int eeAddress = 0;
//      long bnoID;
//      bno.getSensor(&sensor);
//      eeAddress += sizeof(long);
//      EEPROM.get(eeAddress, calibrationData);
//      displaySensorOffsets(calibrationData);
      delay(3500);
    }
  }
  
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
  ESP_DATA_SEND_PACKET.Acc_X = Accel.x();
  ESP_DATA_SEND_PACKET.Acc_Y = Accel.y();
  ESP_DATA_SEND_PACKET.Acc_Z = Accel.z();
  ESP_DATA_SEND_PACKET.Gyro_X = Gyro.x();
  ESP_DATA_SEND_PACKET.Gyro_Y = Gyro.y();
  ESP_DATA_SEND_PACKET.Gyro_Z = Gyro.z();
  ESP_DATA_SEND_PACKET.Mag_X = Mag.x();
  ESP_DATA_SEND_PACKET.Mag_Y = Mag.y();
  ESP_DATA_SEND_PACKET.Mag_Z = Mag.z();
  ESP_DATA_SEND_PACKET.Quat_W = quat.w();
  ESP_DATA_SEND_PACKET.Quat_X = quat.x();
  ESP_DATA_SEND_PACKET.Quat_Y = quat.y();
  ESP_DATA_SEND_PACKET.Quat_Z = quat.z();

}

int Water_Sensor_Update(){

  if(analogRead(Water_Sensor)>50){
//    Serial.println("Minimal water detected");
    strip.setPixelColor(0,50,0,0);
    strip.show();
    return 1;
  }
  if(analogRead(Water_Sensor)>200){
//    Serial.println("More water detected");
    strip.setPixelColor(0,250,0,0);
    strip.show();
    return 2;
  }
  else{
//    Serial.println("No water detected");
    strip.setPixelColor(0,0,50,0);
    strip.show();
    return 0;
  }
}

float Battery_Check(){
// put your main code here, to run repeatedly:
  float Y = 0; // reset the Y variable
  for (int i = 1;i<=100;i++){ // For 100 cycles
    float X = analogRead(Battery_Monitor); // Measure the pin
    X = X*(3.3/4095); // Convert from binary to actual voltage
    X = (X*(R1+R2))/R2; // Back solve the voltage divider
    Y = Y+X; // And add up all the samples
  }
  Y = Y/100; // Find average of all the samples
  Y = ((1/1.1099)*Y)+1.840127; // Apply a linear offset to deal with the non linear response of the Esp32 ADC
  ESP_DATA_SEND_PACKET.Batt_Voltage = Y;
}
