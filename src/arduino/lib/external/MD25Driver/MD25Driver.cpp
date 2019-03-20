/*
 * MD25Driver.cpp - Library for driving the MD25 board
 *
 *       Motors : MD25 with I2C           
 *
 *               by
 *     Davide Brugali, June 08, 2015.
 */

#include "Arduino.h"
#include <Wire.h>
#include "MD25Driver.h"

MD25Driver::MD25Driver() {
}

void MD25Driver::initialize() {
  Wire.begin();
  encoderReset(); // Calls a function that sets the encoder values to 0
}

// Function to stop motors
void MD25Driver::stopMotors() {
  setMotorSpeed(SPEED_LEFT, 0);  
  setMotorSpeed(SPEED_RIGHT, 0);  
}

// Drive motor at the specified speed
// 0:backward full speed    128:STOP    255:forward full speed
// -128:backward full speed   0:STOP    127:forward full speed
void MD25Driver::setMotorSpeed(byte address, int motor_speed) {
    if(motor_speed < -128)
	motor_speed = -128;
    else if(motor_speed > 127)
	motor_speed = 127;   

    motor_speed = 128 + motor_speed;
 
    Wire.beginTransmission(MD25ADDRESS);                    
    Wire.write(address);
    Wire.write(motor_speed);                                           
    Wire.endTransmission();  
}


void MD25Driver::setSpeed(int left, int right) {
	setMotorSpeed(SPEED_LEFT, -left);
	setMotorSpeed(SPEED_RIGHT, -right);
}

   

// This function sets the encoder values to 0
void MD25Driver::encoderReset() {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ENCODER_RESET);
  Wire.write(0x20);                                         // Putting the value 0x20 to reset encoders
  Wire.endTransmission(); 
}

// Function to read the value of an encoder as a long    
long MD25Driver::readEncoder(byte address) {

  Wire.beginTransmission(MD25ADDRESS);                      // Send byte to get a reading from encoder 1
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  long poss = Wire.read();                                  // First byte HH.
  poss <<= 8;
  poss += Wire.read();                                      // Second byte HL
  poss <<= 8;
  poss += Wire.read();                                      // Third byte LH
  poss <<= 8;
  poss  +=Wire.read();                                      // Fourth byte LL

  return(poss);
}


long MD25Driver::readEncoder_L() {
  return readEncoder(ENCODER_LEFT);
}

long MD25Driver::readEncoder_R() {
  return readEncoder(ENCODER_RIGHT);
}

// Function to read battery volts as a single byte
int MD25Driver::volts(){                                               
  Wire.beginTransmission(MD25ADDRESS);                      // Send byte to read volts
  Wire.write(VOLTREAD);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 1);
  while(Wire.available() < 1);                               
  return Wire.read();
}


