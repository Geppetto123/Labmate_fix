/**********************************************************************************************
 * Lib:		Arduino LabmateProtocol Library - Version 1.5.0
 * Date: 	23th November 2018
 *
 * by 		Veronica Nicla Cicchella 	<veronicanicla.cicchella@gmail.com>
 *			Roberto	Conca				<robertoconca1993@gmail.com>
 *			Marco Vitetta				<vito3200@hotmail.it>
 *
 *
 * This Library is licensed under a GPLv3 License
  **********************************************************************************************/
#include "LabmateProtocol.h"
#include <string.h>

using namespace std;

LabmateProtocol* LabmateProtocol::singleton = NULL;

/**************************************
 * Returns an istance of LabmateProtocol
 **************************************/
LabmateProtocol* LabmateProtocol::instance(){
	if(NULL == singleton)
		singleton = new LabmateProtocol();
	return singleton;
}

/**************************************
 * Constructor
 **************************************/
LabmateProtocol::LabmateProtocol() {
	rightPerc 	= 0;
	leftPerc  	= 0;
	rightDir  	= 1;
	leftDir   	= 1;
	angularV  	= 0;
	linearV   	= 0;
	angularPerc = 0;
	linearPerc  = 0;
	twistByte	= 0;
	odometryByte= 0;
	x 			= 0;
	y			= 0;
	theta		= 0;
	validTwist  = false;
	validOdometry= false;
}

/********************************************************************************************
 * Sets the twist values and does boxing/unboxing of the package twist
 ********************************************************************************************/
bool LabmateProtocol::setTwist(double linearVelocity, double angularVelocity){

	double omegaR = (linearVelocity + (angularVelocity*WHEELS_DISTANCE/2))/WHEELS_RADIUS;
	double omegaL = (linearVelocity - (angularVelocity*WHEELS_DISTANCE/2))/WHEELS_RADIUS;
	
	if(omegaR >0)
		rightDir = 1;  // senso ORARIO
	else{
		omegaR = - omegaR;
		rightDir = 0;
	}
	
	if(omegaL >0)
		leftDir = 1;
	else{
		omegaL = - omegaL;
		leftDir = 0;
	}	
	
	if(omegaR > MAX_MOTOR_SPEED || omegaL > MAX_MOTOR_SPEED)   
		return false;
	
	omegaRight = omegaR;
	omegaLeft = omegaL;
	
	linearV = linearVelocity;
	angularV = angularVelocity;
	
	uint8_t lin_dir, ang_dir;
	
	if(linearVelocity >= 0)  lin_dir = 1;    // 1: AVANTI 			0:INDIETRO
	else {
		linearVelocity = - linearVelocity;
		lin_dir = 0;
	}
	
	if(angularVelocity >= 0)  ang_dir = 1;    // 1: ORARIO 			0:ANTIORARIO
	else {
		angularVelocity = - angularVelocity;
		ang_dir = 0;
	}

	rightPerc = (omegaR*100)/MAX_MOTOR_SPEED;
	leftPerc = (omegaL*100)/MAX_MOTOR_SPEED;
	
	uint8_t lin_perc = (linearVelocity*100)/MAX_LINEAR_VEL;
	uint8_t ang_perc = (angularVelocity*100)/MAX_ANGULAR_VEL;

	uint8_t lin_arr[8];
	uint8_t	ang_arr[8];

	dec_bin(lin_perc);
	memcpy(lin_arr,conversionArray,8); // copiamo in arr_v1 la conversione in decimale che e stata salvata in conversionArray
    dec_bin(ang_perc);
	memcpy(ang_arr,conversionArray,8);

	/* IMPACCHETTAMENTO DEI TRE BYTE */
	
	uint8_t byte1[8];
	uint8_t byte2[8];
	uint8_t byte3[8];

	byte1[7]=0;
	byte1[6]=0;
	
	byte3[7]=1;
	byte3[6]=0;
	
	byte2[7]=0;
	byte2[6]=1; 
	byte2[5]=lin_arr[1];
	byte2[4]=lin_arr[0];	
	byte2[3]=lin_dir;
	byte2[2]=ang_arr[1];  
	byte2[1]=ang_arr[0];		
	byte2[0]=ang_dir;

	for(int i=5; i>=0; i--){
        byte1[i]=lin_arr[i+2];
        byte3[i]=ang_arr[i+2];
	}

	twist[0] = bin_dec(byte1);
	twist[1] = bin_dec(byte2);
	twist[2] = bin_dec(byte3);
	validTwist = true;
	return true;
}

/****************************************************************************
 * Controls each byte of twist and returns twist package to send to Arduino
 ****************************************************************************/
bool LabmateProtocol::setTwistByte(uint8_t byte){
	if(byte <= 63){
	    twist[0] = byte;
		twistByte=1;
		validTwist= false;
	    return true;
	}  
	
	if(1 == twistByte){
		if(byte>=64 && byte<=127) {
			twist[1] = byte;
			twistByte++;
			validTwist = false;
			return true;
		}
	    twistByte = 0;

        return false;
	} 
	
	if(2 == twistByte){
		twistByte = 0;
		if(byte>= 128 && byte<=191){
			twist[2] = byte;
			return parseTwist();
		}
	}
	return false;
}

/***********************************************************************
 * Controls each byte of the odometry and create its package
 ***********************************************************************/ 
bool LabmateProtocol::setOdometryByte(uint8_t byte){
	/* odometryByte è il numero del byte atteso */
    odometry[odometryByte] = byte;
    odometryByte++;
	
	if(odometryByte%5==0){  // se abbiamo ricevuto *multipli di 5 byte* 
		if(odometryByte == 5){  // al 5 ci aspettiamo esattamente la x
			if(odometry[0]>=32 && odometry[0]<=47){
				/* DECODING OF X */
				uint32_t x=0;
				x = x | (odometry[0] & 15);
				uint8_t i;
				for(i=1; i<5; i++){
					x <<= 7;
					x = x | (odometry[i] & 127);
				}
				memmove(&(this->x),&x,FLOAT_BYTES);
				return true;
			}
		} else if (odometryByte == 10){
			if(odometry[5]>=48 && odometry[5]<=63){
				/* DECODING OF Y */
				uint32_t y=0;
				y = y | (odometry[5] & 15);
				uint8_t i;
				for(i=6; i<10; i++){
					y <<= 7;
					y = y | (odometry[i] & 127);
				}
				memmove(&(this->y),&y,FLOAT_BYTES);
				return true;
			}
		} else if (odometryByte == 15){
			if(odometry[10]>=64 && odometry[10]<=79){
				/* DECODING OF THETA */
				uint32_t theta=0;
				theta = theta | (odometry[10] & 15);
				uint8_t i;
				for(i=11; i<15; i++){
					theta <<= 7;
					theta = theta | (odometry[i] & 127);
				}
				memmove(&(this->theta),&theta,FLOAT_BYTES);
				odometryByte = 0;
				validOdometry = true;
				return true;
			}
		}		
	} else {
		if((odometryByte-1)%5==0){
			if(byte<128)
				return true;
		} else {
			if(byte>=128)
				return true;
		}
	}
    odometryByte = 0;

	return false;
}

/**************************************
 * Destroyer
 **************************************/
LabmateProtocol::~LabmateProtocol(){
}

/********************************************************
 * Conversion from decimal to binary
 ********************************************************/
uint8_t* LabmateProtocol::dec_bin(uint8_t dec){
	for(int i=0; i<8; i++){
		conversionArray[i]=dec%2;
		dec=dec/2;
	}
	return conversionArray;
} 

/**********************************************************
 * Conversion from binary to decimal
 **********************************************************/
uint8_t LabmateProtocol::bin_dec(uint8_t* array){
	uint8_t dec=0;
	uint8_t factor = 1;
	
	for(int i=0; i<8; i++){
		dec += array[i]*factor;
		factor *= 2;  // factor <<=1
	}	
	return dec;
}

/***************************************************************
 * Conversion from decimal to binary
 ***************************************************************/
uint8_t* LabmateProtocol::dec_bin_Odo(float dec){	// da DOUBLE a 32 BITS
	uint8_t arr[FLOAT_BYTES];

	memmove(arr,&dec,FLOAT_BYTES);
	for(int i=FLOAT_BITS; i--; ){
		conversionArrayOdom[i]= (arr[i/CHAR_BIT] & (1u<<(i%CHAR_BIT)))? 1 : 0;
	}
	return conversionArrayOdom;
}

/***********************************************************************
 * Returns true if the twist is valid and creates the twist package 
 ************************************************************************/
bool LabmateProtocol::getTwist(uint8_t* array){
	if(validTwist){
		array[0]= twist[0];
		array[1]= twist[1];
		array[2]= twist[2];	
		return true;
	}
	return false;
}

/**************************************************
 * Returns true if odometry is correct
 **************************************************/
bool LabmateProtocol::getOdometry(uint8_t* array){
	//if(validOdometry)
		memmove(array, odometry, 15);
	return validOdometry;
}  

bool LabmateProtocol::parseTwist(){
	uint8_t byte1[8];
	dec_bin(twist[0]);
	memmove(byte1, conversionArray, 8);
	uint8_t byte2[8];
	dec_bin(twist[1]);
	memmove(byte2, conversionArray, 8);
	uint8_t byte3[8];
	dec_bin(twist[2]);	
	memmove(byte3, conversionArray, 8);
	
    /*SCOMPATTAMENTO DI BYTE2*/ 
	
	uint8_t fattore=2;
	uint8_t linearPerc =0;
	uint8_t angularPerc =0;
	
    linearPerc += byte2[4];
    linearPerc += byte2[5]* fattore;
    angularPerc += byte2[1];
    angularPerc += byte2[2]* fattore;

    for(int j=0; j<6; j++){
	   fattore = fattore * 2;
       linearPerc += byte1[j]*fattore;
       angularPerc += byte3[j]*fattore;
    }
	
	if(linearPerc> 100 || angularPerc>100) 
		return false;	

    rightDir = byte2[3]; 
    leftDir = byte2[0];
	
	linearPerc= linearPerc;
	angularPerc= angularPerc;
	
	linearV = (linearPerc*MAX_LINEAR_VEL)/100;
	angularV = (angularPerc*MAX_ANGULAR_VEL)/100;
	
	if(rightDir==0)
		linearV = - linearV;
	if(leftDir==0)
		angularV = - angularV;
	
	return setTwist(linearV, angularV);
} 

/****************************************************************************************************************************************************
 * Sets x value and creates the package to send to miniPc from Arduino
 ****************************************************************************************************************************************************/
void LabmateProtocol::setOdometryX(float x){
	
	singleton->x=x;
	uint8_t x_bits[FLOAT_BITS];

	dec_bin_Odo(x);
	memmove(x_bits,conversionArrayOdom,FLOAT_BITS); // copiamo in arr_v1 la conversione in decimale che e stata salvata in conversionArray
	
	uint8_t byte[8];	
	
	 /* SETTIAMO I BIT DI FLAG + 3 BIT RELATIVI ALLA VARIABILE */
	 
	byte[7] = 0;
	byte[6] = 0;
	byte[5] = 1;
	byte[4] = 0;	
    byte[3] = x_bits[FLOAT_BITS-1];
	byte[2] = x_bits[FLOAT_BITS-2];
	byte[1] = x_bits[FLOAT_BITS-3];
	byte[0] = x_bits[FLOAT_BITS-4];	
	odometry[0]= bin_dec(byte);
	
	byte[7]=1;
	int i;
	for(i=6; i>=0; i--){
		byte[i]= x_bits[FLOAT_BITS-11+i];
	}
	odometry[1]= bin_dec(byte);
		
	byte[7]=1;
	for(i=6; i>=0; i--){
		byte[i]= x_bits[FLOAT_BITS-18+i];
	}
	odometry[2]= bin_dec(byte);	
	
	byte[7]=1;
	for(i=6; i>=0; i--){
		byte[i]= x_bits[FLOAT_BITS-25+i];
	}
	odometry[3]= bin_dec(byte);	

	byte[7]=1;
	for(i=6; i>=0; i--){
		byte[i]= x_bits[i];
	}
	odometry[4]= bin_dec(byte);		
}

/*****************************************************************************************************************************************************
 * Sets y value and creates the package to send to miniPc from Arduino
 *****************************************************************************************************************************************************/
void LabmateProtocol::setOdometryY(float y){
	
	singleton->y=y;
	uint8_t y_bits[FLOAT_BITS];

	dec_bin_Odo(y);
	memmove(y_bits,conversionArrayOdom,FLOAT_BITS); // copiamo in arr_v1 la conversione in decimale che e stata salvata in conversionArray
	
	uint8_t byte[8];	
	
	 /* SETTIAMO I BIT DI FLAG + 3 BIT RELATIVI ALLA VARIABILE */
	 
	byte[7] = 0;
	byte[6] = 0;
	byte[5] = 1;
	byte[4] = 1;	
    byte[3] = y_bits[FLOAT_BITS-1];
	byte[2] = y_bits[FLOAT_BITS-2];
	byte[1] = y_bits[FLOAT_BITS-3];
	byte[0] = y_bits[FLOAT_BITS-4];	
	odometry[5]= bin_dec(byte);
	
	byte[7]=1;
	int i;
	for(i=6; i>=0; i--){
		byte[i]= y_bits[FLOAT_BITS-11+i];
	}
	odometry[6]= bin_dec(byte);
		
	byte[7]=1;
	for(i=6; i>=0; i--){
		byte[i]= y_bits[FLOAT_BITS-18+i];
	}
	odometry[7]= bin_dec(byte);	
	
	byte[7]=1;
	for(i=6; i>=0; i--){
		byte[i]= y_bits[FLOAT_BITS-25+i];
	}
	odometry[8]= bin_dec(byte);	

	byte[7]=1;
	for(i=6; i>=0; i--){
		byte[i]= y_bits[i];
	}
	odometry[9]= bin_dec(byte);	
}

/**********************************************************************************************************************************************
 * Sets theta value and creates the package to send to miniPc from Arduino
 **********************************************************************************************************************************************/
void LabmateProtocol::setOdometryTheta(float theta){
	
	singleton->theta=theta;
	uint8_t theta_bits[FLOAT_BITS];

	dec_bin_Odo(theta);
	memmove(theta_bits,conversionArrayOdom,FLOAT_BITS); // copiamo in arr_v1 la conversione in decimale che e stata salvata in conversionArray
	
	uint8_t byte[8];	
	
	 /* SETTIAMO I BIT DI FLAG + 3 BIT RELATIVI ALLA VARIABILE */
	 
	byte[7] = 0;
	byte[6] = 1;
	byte[5] = 0;
	byte[4] = 0;	
    byte[3] = theta_bits[FLOAT_BITS-1];
	byte[2] = theta_bits[FLOAT_BITS-2];
	byte[1] = theta_bits[FLOAT_BITS-3];
	byte[0] = theta_bits[FLOAT_BITS-4];	
	odometry[10]= bin_dec(byte);
	
	byte[7]=1;
	int i;
	for(i=6; i>=0; i--){
		byte[i]= theta_bits[FLOAT_BITS-11+i];
	}
	odometry[11]= bin_dec(byte);
	
	byte[7]=1;
	for(i=6; i>=0; i--){
		byte[i]= theta_bits[FLOAT_BITS-18+i];
	}
	odometry[12]= bin_dec(byte);	
	
	byte[7]=1;
	for(i=6; i>=0; i--){
		byte[i]= theta_bits[FLOAT_BITS-25+i];
	}
	odometry[13]= bin_dec(byte);	

	byte[7]=1;
	for(i=6; i>=0; i--){
		byte[i]= theta_bits[i];
	}
	odometry[14]= bin_dec(byte);	
}

/**************************************
 * Returns the angular velocity 
 **************************************/
double LabmateProtocol::getAngularV(){
	return angularV;
}

/**************************************
 * Returns the linear velocity 
 **************************************/
double LabmateProtocol::getLinearV(){
	return linearV;
}

/**************************************
 * Returns the rate of angular velocity
 **************************************/
uint8_t LabmateProtocol::getAngularPerc(){
	return angularPerc;
}

/**************************************
 * Returns the rate of linear velocity
 **************************************/	
uint8_t LabmateProtocol::getLinearPerc(){
	return linearPerc;
}

/******************************************
 * Returns the rate of right wheel velocity
 ******************************************/
uint8_t LabmateProtocol::getRightPerc(){
	return rightPerc;
}

/******************************************
 * Returns the rate of left wheel velocity
 ******************************************/
uint8_t LabmateProtocol::getLeftPerc(){
	return leftPerc;
}

/**********************************************************
 * Returns the direction of right wheel
 **********************************************************/
uint8_t LabmateProtocol::getRightDir(){
	return rightDir;
}

/**********************************************************
 * Returns the direction of left wheel 
 **********************************************************/
uint8_t LabmateProtocol::getLeftDir(){
	return leftDir;
}

/**************************************
 * Returns the x value of odometry
 **************************************/
float LabmateProtocol::getX(){
	return x;
}

/**************************************
 * Returns the y value of odometry
 **************************************/
float LabmateProtocol::getY(){
	return y;
}

/**************************************
 * Returns the theta value of odometry
 **************************************/
float LabmateProtocol::getTheta(){
	return theta;
}

/**************************************
 * Returns true if twist is valid
 **************************************/
bool LabmateProtocol::isTwistValid(){
	return validTwist;
}

/**************************************
 * Returns true if odometry is valid
 **************************************/
bool LabmateProtocol::isOdometryValid(){
	return validOdometry;
}
