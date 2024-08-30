// Libraries and Global Variables
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// ******************************************
// General information
// ******************************************
// IMU testing. No library used, except Wire to use the I2C
// ******************************************
// Variable declaration
// ******************************************

// ESC Control
// Potentiometer
const int PotPin_Left = A0;
const int PotPin_Right = A1;
double throttle = 1200;     //initial value of throttle to the motors

// ESC
Servo ESC_Left;  // create servo object to control the ESC 1
Servo ESC_Right;  // create servo object to control the ESC 2
int PotVal_Left, PotVal_Right;
float Angle_Left, Angle_Right;

// IMU parameters
int16_t Acc_raw_X, Acc_raw_Y, Acc_raw_Z;
int16_t Gyro_raw_X, Gyro_raw_Y, Gyro_raw_Z;
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float GyroX_calib, GyroY_calib, GyroZ_calib;
float AccX_calib, AccY_calib, AccZ_calib;

long unsigned prev_time_mpu, dt_mpu;
float rad_to_deg = 180 / 3.141592654;
float Acc_angle[3];
float Gyro_angle[3] = { 0, 0, 0 };
float Total_angle[3] = { 0, 0, 0 };
int i;

// PID Control
double Setpoint, Input, Output;
double Kp = 9.255, Ki = 0.025, Kd = 2.745;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Serial configuration parameters
unsigned long curr_time = 0, prev_time_com = 0, dt_com = 50000;  // time interval in us

// User Input
int controlMode;

// ******************************************
// Function declaration
// ******************************************
void SerialDataWrite();
void gyro_signals();
void acc_signals();
float floatMap(float, float, float, float, float);

// ******************************************
// void setup
// ******************************************
void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  curr_time = micros();

  Serial.println("You have 2 seconds to align the MPU vertically");
  delay(2000);
  // MPU preparation and calibration
  Wire.setClock(400000);  // set wire clock to the max (400kHz)
  Wire.begin();           // Start the wire comunication
  delay(100);
  Wire.beginTransmission(0x68);  // Start the communication with the MPU6050
  Wire.write(0x6B);              // Start the gyro in power mode
  Wire.write(0x00);              // Set the requested starting register
  Wire.endTransmission();        // End the transmission

  // PID Setup  
  Setpoint = 0;
  myPID.SetOutputLimits(-800, 800);
  myPID.SetMode(AUTOMATIC);

  // Manual Setup
  ESC_Left.attach(9, 1000, 2000);   // (pin, min pulse width, max pulse width in microseconds)
  ESC_Right.attach(10, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
  //analogReadResolution(12);     // Only for microctrl with 12bit ADC
  ESC_Left.write(0);  // Send the signal to the ESC
  ESC_Right.write(0);  // Send the signal to the ESC

  // MPU6050 Gyro Calibration
  Serial.println("Begin MPU Gyro Calibration");
  for (i = 0; i < 2000; i++) {
    gyro_signals();
    GyroX_calib += GyroX;
    GyroY_calib += GyroY;
    GyroZ_calib += GyroZ;
  }
  GyroX_calib /= 2000;
  GyroY_calib /= 2000;
  GyroZ_calib /= 2000;
  // MPU6050 Acc Calibration
  Serial.println("Begin MPU Acc Calibration");
  for (i = 0; i < 2000; i++) {
    acc_signals();
    AccX_calib += AccX;
    AccY_calib += AccY;
    AccZ_calib += AccZ;
  }
  // For the calibration of the accelerometer, check what is the direction of the MPU. For me, g is in the negative X-axis
  // Because the gravitational acceleration should not be part of the calibration
  AccX_calib = AccX_calib / 2000 - 1;
  AccY_calib = AccY_calib / 2000;
  AccZ_calib = AccZ_calib / 2000;

  Serial.println("MPU calibrated. Press any key to start the main program.");
  while (!Serial.available())
    ;
  Serial.read();

  // Prompt user
  Serial.println("Choose Control Mode: ");
  Serial.println("1 - PID Control");
  Serial.println("2 - Manual Control");

  // Wait for input
  while(!Serial.available()) { }

  // Read user's choice
  controlMode = Serial.parseInt();

  // Validate input
  while(controlMode != 1 && controlMode != 2){
    Serial.println("Invalid mode. Please enter 1 or 2.");
    while(!Serial.available()) { }
    controlMode = Serial.parseInt();
  }

  // PID Control Mode Setup
  if(controlMode == 1) {
    Serial.println("Warning!!");
    Serial.println("After you press Enter, you have 2 seconds to get ready (or to insert the power) before the motors start to spin. Be careful!!");

    // Wait for input
    while(!Serial.available());
    Serial.read();

    // In order to start up the ESCs we have to send a min value
    // of PWM to them before connecting the battery. Otherwise
    // the ESCs won't start up or enter in the configure mode.
    // Min: 1000 - Max: 2000
    ESC_Left.writeMicroseconds(1000);  // Send the signal to the ESC
    ESC_Right.writeMicroseconds(1000);  // Send the signal to the ESC
    delay(2000);
  }
  
  // Manual Control Mode Setup (User controls the motors using 2 Pots)
  else {
    ESC_Left.write(0);  // Send the signal to the ESC
    ESC_Right.write(0);  // Send the signal to the ESC
  }
}

void loop() {

  gyro_signals();  // Get gyro data GyroX, GyroY, GyroZ
  acc_signals();   // Get acc data AccX, AccY, AccZ

  GyroX -= GyroX_calib;
  GyroY -= GyroY_calib;
  GyroZ -= GyroZ_calib;
  AccX -= AccX_calib;
  AccY -= AccY_calib;
  AccZ -= AccZ_calib;

  dt_mpu = (curr_time - prev_time_mpu);
  prev_time_mpu = curr_time;

  Acc_angle[1] = atan2(AccZ, sqrt(pow(AccY, 2) + pow(AccX, 2))) * rad_to_deg;
  Acc_angle[2] = atan2(-AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * rad_to_deg;

  Gyro_angle[0] += GyroX * dt_mpu / 1000000;
  Gyro_angle[1] += GyroY * dt_mpu / 1000000;
  Gyro_angle[2] += GyroZ * dt_mpu / 1000000;

  Total_angle[0] = Gyro_angle[0];
  Total_angle[1] = .98 * (Total_angle[1] + GyroY * dt_mpu / 1000000) + .02 * Acc_angle[1];
  Total_angle[2] = .98 * (Total_angle[2] + GyroX * dt_mpu / 1000000) + .02 * Acc_angle[2];

  // PID Control Mode
  if (controlMode == 1) {
    // Input the Roll angle
    Input = Total_angle[2];

    // Calculate the Output
    myPID.Compute();

    // Add/Subtract the Output from the Initial Signal  
    PotVal_Left = throttle + Output;
    PotVal_Right = throttle - Output;
   
    /* PWM limiter */
    // To keep the ESCs in range of 1000 - 2000 us
    //Right
    if(PotVal_Right < 1000) PotVal_Right = 1000;
    if(PotVal_Right > 2000) PotVal_Right = 2000;

    //Left
    if(PotVal_Left < 1000) PotVal_Left = 1000;
    if(PotVal_Left > 2000) PotVal_Left = 2000;

    // Send the signal to the ESCs
    ESC_Left.writeMicroseconds(PotVal_Left);
    ESC_Right.writeMicroseconds(PotVal_Right);
  }

  // Manual Control Mode
  else {
    // Read value from Potentiometer
    PotVal_Left = analogRead(PotPin_Left);
    PotVal_Right = analogRead(PotPin_Right);

    // Change the value range of the potentiometer to range of ESC signal
    Angle_Left = floatMap(PotVal_Left, 0, 4095, 1000, 2000);
    Angle_Right = floatMap(PotVal_Right, 0, 4095, 1000, 2000);
    
    // Send the signal to the ESCs
    ESC_Left.writeMicroseconds(Angle_Left);  // Send the signal to the ESC
    ESC_Right.writeMicroseconds(Angle_Right);  // Send the signal to the ESC
  }
  // Serial write
  curr_time = micros();
  if (curr_time - prev_time_com >= dt_com) {
    prev_time_com += dt_com;
    SerialDataWrite();
  }
}

// ******************************************
// Function definitions
// ******************************************
void SerialDataWrite() {
  Serial.println(String(curr_time / 1000) + " Left: " + String(PotVal_Left) + ", Right: " + String(PotVal_Right) + ", Pitch: " + String(Total_angle[1]) + ", Roll: " + String(Total_angle[2]));
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);  // Start the communication with the MPU6050
  Wire.write(0x1A);              // Enable Low-pass filter
  Wire.write(0x05);
  Wire.endTransmission();  // End Transmission
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  // Change Gyro sensitivity
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);  // Access register storing Gyro data
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);  // Read the Gyro data
  Gyro_raw_X = Wire.read() << 8 | Wire.read();
  Gyro_raw_Y = Wire.read() << 8 | Wire.read();
  Gyro_raw_Z = Wire.read() << 8 | Wire.read();
  // GyroX = (float)Gyro_raw_X / 65.5;
  // GyroY = (float)Gyro_raw_Y / 65.5;
  // GyroZ = (float)Gyro_raw_Z / 65.5;

  // Change the sensitivity of sensor
  GyroX = (float)Gyro_raw_X / 131.0;
  GyroY = (float)Gyro_raw_Y / 131.0;
  GyroZ = (float)Gyro_raw_Z / 131.0;
}

void acc_signals(void) {
  Wire.beginTransmission(0x68);  // Start the communication with the MPU6050
  Wire.write(0x1A);              // Enable Low-pass filter
  Wire.write(0x05);              // Enable Low-pass filter - 10Hz-ish
  Wire.endTransmission();        // End Transmission
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Access register storing Acc data
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);  // Read the Acc data
  Acc_raw_X = Wire.read() << 8 | Wire.read();
  Acc_raw_Y = Wire.read() << 8 | Wire.read();
  Acc_raw_Z = Wire.read() << 8 | Wire.read();
  AccX = (float)Acc_raw_X / 16384.0;
  AccY = (float)Acc_raw_Y / 16384.0;
  AccZ = (float)Acc_raw_Z / 16384.0;
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}