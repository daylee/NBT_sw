// The script below is the simplest code that can accomplish all needed goals for a tabletop test

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0x6B, 0x45, 0x44};
//Change to the MAC address of the base station board being used
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <Chrono.h>
Chrono chrono_ET;

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

#define Water_Sensor A2
#define Signal_LED_Data 21
#define Battery_Monitor A3

// Define the sample rate and the I2C address of the sensor (can be adjusted for using more than 1 at once)
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);

// Indicator LEDs
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip(5,Signal_LED_Data); // Change to indicate the number of LEDS used



// Library and setup for Hardware Serial 
#include <EasyTransfer.h>
#define HWSERIAL Serial1
EasyTransfer ET;

typedef struct struct_message_OUT{
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
} struct_message_OUT;


typedef struct struct_message_IN{
  float CoMA1_X; // Position to move to in mm
  float CoMA2_X;
  float CoMA3_X;
  float CoMA4_X;
  float RW_A_RPM; // Commanded RPM for the reaction wheels
  float RW_B_RPM;
  float RW_C_RPM;
} struct_message_IN;

float COMA1;
float COMA2;
float COMA3;
float COMA4;
float RW1;
float RW2;
float RW3;


struct_message_IN INCOMING;
struct_message_OUT OUTGOING;
struct_message_IN OUTGOING_HW;

String success;

void OnDataSent(const uint8_t *mac_add,esp_now_send_status_t status){
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
//    success = "Delivery Success :) CB";
  }
  else{
//    success = "Delivery Fail :( CB";
  }
  // Turn the above off once it's working correctly. The basestation doesn't need to do anything on data send
  // Alternatively, print it to the LCD screen
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&INCOMING, incomingData, sizeof(INCOMING));
  COMA1 = INCOMING.CoMA1_X;
  COMA2 = INCOMING.CoMA2_X;
  COMA3 = INCOMING.CoMA3_X;
  COMA4 = INCOMING.CoMA4_X;
  RW1 = INCOMING.RW_A_RPM;
  RW2 = INCOMING.RW_B_RPM;
  RW3 = INCOMING.RW_C_RPM;
//  commandPrintOut();
  OUTGOING_HW.CoMA1_X = COMA1;
  OUTGOING_HW.CoMA2_X = COMA2;
  OUTGOING_HW.CoMA3_X = COMA3;
  OUTGOING_HW.CoMA4_X = COMA4;
  OUTGOING_HW.RW_A_RPM = RW1;
  OUTGOING_HW.RW_B_RPM = RW2;
  OUTGOING_HW.RW_C_RPM = RW3;
//  Serial.println(OUTGOING_HW.CoMA1_X);
  
  
}

void setup(){
  
  Serial.begin(9600);
  ET.begin(details(OUTGOING_HW),&Serial1);
  HWSERIAL.begin(9600);
  
  if(!bno.begin()){
    Serial.println("No BNO sensor detected");
    while(1);    
  }
  delay(100);
  bno.setExtCrystalUse(true);// Using the crystal of the Microcontroller rather than the BNO crystal is more accurate

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
  
  // LED setup
  strip.begin();
  strip.show();
  strip.setPixelColor(0,0,0,0);
  strip.show();
  
  strip.setPixelColor(0,0,50,0);
  strip.show(); // Turn on an LED to show that the system is booted

  // Pinmodes
  pinMode(Water_Sensor,INPUT);
  pinMode(Signal_LED_Data,OUTPUT);
  pinMode(Battery_Monitor,INPUT);
}

void loop(){
  strip.setPixelColor(0,100,100,100);
  strip.show();
  // Turn on a white LED to indicate main loop
  IMU_UPDATE();
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &OUTGOING, sizeof(OUTGOING));
  if (chrono_ET.hasPassed(1000)){
    ET.sendData(); // This is because we want to make sure we don't overflow the buffer on the Teensy
    chrono_ET.restart();
    commandPrintOut();
  }

}

void commandPrintOut(){
  Serial.println("Command print out ");
  Serial.print("COMA1: ");
  Serial.print(COMA1);
  Serial.print(" COMA2: ");
  Serial.print(COMA2);
  Serial.print(" COMA3: ");
  Serial.print(COMA3);
  Serial.print(" COMA4: ");
  Serial.println(COMA4);
  Serial.print(" RW1: ");
  Serial.print(RW1);
  Serial.print(" RW2: ");
  Serial.print(RW2);
  Serial.print(" RW3: ");
  Serial.println(RW3);
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
  OUTGOING.QW = quat.w();
  OUTGOING.QX = quat.x();
  OUTGOING.QY = quat.y();
  OUTGOING.QZ = quat.z();
//  Serial.println("IMU UPDATE");

}
  
