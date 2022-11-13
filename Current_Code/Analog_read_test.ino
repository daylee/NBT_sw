// Analog test

#define Pin 34


float R1 = 15*1000;
float R2 = 1*1000;

// In actuality, R1 = 10K + 2.2K + 3X 500 ohm resistors. = ~13.7K ohms
// This was found such that without the linear offset applied below, 19.2V in = a reading of 19.2V, which is the cutoff voltage

void setup() {
  // put your setup code here, to run once:
  pinMode(Pin,INPUT); // Set pin mode
  Serial.begin(9600); // Begin serial communication

}

void loop() {
  // put your main code here, to run repeatedly:
  float Y = 0; // reset the Y variable
  for (int i = 1;i<=100;i++){ // For 100 cycles
    float X = analogRead(Pin); // Measure the pin
    X = X*(3.3/4095); // Convert from binary to actual voltage
    X = (X*(R1+R2))/R2; // Back solve the voltage divider
    Y = Y+X; // And add up all the samples
  }
  Y = Y/100; // Find average of all the samples
  Y = ((1/1.1099)*Y)+1.840127; // Apply a linear offset to deal with the non linear response of the Esp32 ADC
  Serial.println(Y); // Print out the 
  delay(100);
}
