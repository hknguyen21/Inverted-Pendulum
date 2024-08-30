// Libraries and Global Variables
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

#define MAX_SIGNAL 2000

#define MIN_SIGNAL 1000

#define MOTOR_PIN 9

 

Servo ESC;

 
void setup() {
  // analogReadResolution(12); // Only for microctrl with 12bit ADC

  Serial.begin(9600);

  delay(10);

  Serial.println("This program will start the calibration. Remove the 5V wire. ");

  Serial.println("This program will start the ESC. Press any key to start. ");

 

  // Wait for input

  while (!Serial.available());

  Serial.read();

 

  ESC.attach(MOTOR_PIN, MIN_SIGNAL, MAX_SIGNAL);

  ESC.writeMicroseconds(MAX_SIGNAL);

 

  Serial.print("Now writing maximum output: (");Serial.print(MAX_SIGNAL);Serial.print(" us in this case)");Serial.print("\n");

  Serial.println("Turn on power source, then wait 2 seconds and press any key.");

 


  // Wait for input

  while (!Serial.available());

  Serial.read();

 

  // Send min output

  Serial.println("\n");

  Serial.println("\n");

  Serial.print("Sending minimum output: (");Serial.print(MIN_SIGNAL);Serial.print(" us in this case)");Serial.print("\n");

  ESC.writeMicroseconds(MIN_SIGNAL);

  Serial.println("The ESC is calibrated");

  Serial.println("Connect the 5V wire and press any key.");

 

  // Wait for input

  while (!Serial.available());

  Serial.read();

}

 

void loop() {
  int cmd = map(analogRead(A0), 0, 4095, 0, 180);
  ESC.write(cmd);
}