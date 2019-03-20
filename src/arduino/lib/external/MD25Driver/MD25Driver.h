/*
 * MD25Driver.h - Library for driving the MD25 board
 *
 *       Motors : MD25 with I2C           
 *
 *               by
 *     Davide Brugali, June 08, 2015.
 */


#ifndef MD25Driver_h
#define MD25Driver_h

#include "Arduino.h"

#define MD25ADDRESS         0x58                    // Address of the MD25
#define SPEED_RIGHT         (byte)0x00              // Byte to send speed to left motor
#define SPEED_LEFT          0x01                    // Byte to send speed to right motor
#define ENCODER_LEFT        0x02                    // Byte to read motor encoder 1
#define ENCODER_RIGHT       0x06                    // Byte to read motor encoder 2
#define ENCODER_RESET       0x10                    // Byte to reset the encoders
#define VOLTREAD            0x0A                    // Byte to read battery volts

class MD25Driver {
  private:
   void setMotorSpeed(byte address, int motor_speed);   // set the motor speed 
                                                    // -128:backward full speed   0:STOP    
                                                    //  127:forward full speed
   long readEncoder(byte address);                 // return the encoder value
    

  public:
    MD25Driver();
    void initialize();                              // init the communication with
                                                    // the motors and reset the encoders

    void setSpeed(int left, int right);
    void stopMotors();                              // stop the motors

    long readEncoder_L();
    long readEncoder_R();
    void encoderReset(); 			    // reset the encoder values to 0

    int  volts();                                   // return the battery level
};


#endif
