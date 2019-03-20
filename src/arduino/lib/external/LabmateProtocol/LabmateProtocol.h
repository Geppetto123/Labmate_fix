/**********************************************************************************************
 * Lib:		Arduino LabmateProtocol Library - Version 1.0.0
 * Date: 	23th November 2018
 *
 * by 		Veronica Nicla Cicchella 	<veronicanicla.cicchella@gmail.com>
 *			Roberto	Conca				<robertoconca1993@gmail.com>
 *			Marco Vitetta				<vito3200@hotmail.it>
 *
 *
 * This Library is licensed under a GPLv3 License
  **********************************************************************************************/
#ifndef LABMATE_PROTOCOL_H
#define LABMATE_PROTOCOL_H

#include <stdint.h>	
#include <string.h>
#include <limits.h>

/*! \def DOUBLE_BYTES
    \brief Dimension of double.
   
    Costant that defines the size of doubles expressed in bytes
*/#define DOUBLE_BYTES			sizeof(double)

/*! \def DOUBLE_BITS
    \brief Number of bits of a double.

*/#define DOUBLE_BITS			DOUBLE_BYTES*CHAR_BIT

/*! \def FLOAT_BYTES
    \brief Dimension of float.
   
    Costant that defines the size of floats expressed in bytes.
*/#define FLOAT_BYTES			sizeof(float)

/*! \def FLOAT_BITS
    \brief Number of bits of a float.

*/#define FLOAT_BITS			FLOAT_BYTES*CHAR_BIT

/*! \def MAX_ANGULAR_VEL
    \brief Maximum angular velocity.
   
    Maximum angular velocity for Labmate, expressed in rad/sec.
*/#define MAX_ANGULAR_VEL    	6.8

/*! \def MAX_LINEAR_VEL
    \brief Maximum linear velocity.
   
    Maximum linear velocity for Labmate, expressed in m/sec.
*/#define MAX_LINEAR_VEL     	1.13

/*! \def MAX_MOTOR_SPEED
    \brief Maximum velocity of the motor.
   
    Expressed in rad/sec. MAX_MOTOR_SPEED = MAX_IMPULSES_PER_SECOND * MEASURE_RESOLUTION = ((MAX_IMPULSES_PER_PERIOD/PERIOD)/4)*((2*pi)/10000)) 
*/#define MAX_MOTOR_SPEED    	15.7
													
/*! \def WHEELS_DISTANCE
    \brief Lenght of the robot.
   
    Distance between the two centers of the wheels, expressed in meters.
*/#define WHEELS_DISTANCE 		0.33

/*! \def WHEELS_RADIUS
    \brief Radius of the wheels.
   
    Radius of the wheels, expressed in meters.
*/#define WHEELS_RADIUS 		0.0725


/*! \class LabmateProtocol.
	\brief A Labmate Protocol Class.
  
    This class is responsible of implementing the Labmate protocol to let the mini pc communicate with the Arduino board.
*/
class LabmateProtocol {
	public:
		/* FUNCTIONS */
    
		/* SETTERS */
		/*!	\fn bool setTwist(double linearV, double angularV);
			\brief Setter of the twist.

			Takes a linear velocity and angular velocity and returns an boolean value, true if the settings is succesful.
			
			\param linearV a double.
			\param angularV a double.
			
			\return Boolean if sucessful.
		*/bool 	setTwist(double linearV, double angularV);
		
		/*!	\fn void setOdometryX(double x);
			\brief Setter of the x value of odometry.

			Takes the x position and sets this value in the class to calculate the odometry.
			
			\param x a double.
		*/void	setOdometryX(float x);
		
		/*!	\fn void setOdometryY(double y);
			\brief Setter of the y value of odometry.

			Takes the y position and sets this value in the class to calculate the odometry.
			
			\param y a double.
		*/void 	setOdometryY(float y);
		
		/*!	\fn void setOdometryTheta(double theta);
			\brief Setter the theta value of odometry.

			Takes the theta position and sets this value in the class to calculate the odometry.
			
			\param theta a double.
		*/void 	setOdometryTheta(float theta);
		
		/*!	\fn bool 	setTwistByte(uint8_t byte);
			\brief Setter of the twist bytes.

			Takes each single byte of twist and controls it, returns true if all the bytes are valid.
			
			\param byte an integer.
			
			\return Boolean if sucessful settings.
		*/bool 	setTwistByte(uint8_t byte);
		
		/*!	\fn bool setOdometryByte(uint8_t byte);
			\brief Setter of the odometry bytes.

			Takes each single byte of odometry and controls it, returns true if all the bytes are valid.
			
			\param byte an integer.
			
			\return Boolean if sucessful settings.
		*/bool 	setOdometryByte(uint8_t byte);
		
		/* GETTERS */ 
		
		/*!	\fn bool getOdometry(uint8_t* array);
			\brief getOdometry.
			
			Returns a boolean if the array is correct.
			
			\param array a buffer of integer.
			\return Returns a boolean if odometry is correct.
		*/bool    getOdometry(uint8_t* array);
		
		/*!	\fn double 	getAngularV();
			\brief Getter of the angular velocity.
			
			Returns a angular velocity for double.
			
			\return The angular velocity of labmate.
		*/double 	getAngularV();
		
		/*!	\fn double 	getLinearV();
			\brief Getter of the linear velocity.
			
			Returna a linear velocity for double.
			
			\return The linear velocity of labmate.
		*/double 	getLinearV();
		
		/*!	\fn double getX();
			\brief Getter of x value of odometry.
			
			Returns a x position for odometry for double.
			
			\return The x position of labmate.
		*/float  getX();
		
		/*!	\fn double getY();
			\brief Getter of y value of odometry.
			
			Returns a y position for odometry for double.
			
			\return The y position of labmate.
		*/float  getY();
		
		/*!	\fn double getTheta();
			\brief Getter of theta value of odometry.
			
			Returns a theta position for odometry for double.
			
			\return The theta position of labmate.
		*/float  getTheta();
		
		/*!	\fn uint8_t getAngularPerc();
			\brief Getter of angular rate.
			
			Returna an angular velocity rate for integer.
			
			\return The angular velocity rate of labmate.
		*/uint8_t getAngularPerc();
		
		/*!	\fn uint8_t getLinearPerc();
			\brief Getter of linear rate.
			
			Returns a linear velocity rate for integer.
			
			\return The linear velocity rate of labmate.
		*/uint8_t getLinearPerc();
		
		/*!	\fn uint8_t getLeftPerc();
			\brief Getter of left wheel rate.
			
			Returns a left wheel velocity rate for integer.
			
			\return The left wheel velocity rate of labmate.
		*/uint8_t getLeftPerc();
		
		/*!	\fn uint8_t getRightPerc();
			\brief Getter of right wheel rate.
			
			Returns a right wheel velocity rate for integer.
			
			\return The right wheel velocity rate of labmate.
		*/uint8_t getRightPerc();
		
		/*!	\fn uint8_t getLeftDir();
			\brief Getter of left wheel direction.
			
			Returns a left wheel direction for integer.
			
			\return The left wheel direction of labmate.
		*/uint8_t getLeftDir();
		
		/*!	\fn uint8_t getRightDir();
			\brief Getter of right wheel direction.
			
			Returns a right wheel direction for integer.
			
			\return The right wheel direction of labmate.
		*/uint8_t getRightDir();
		
		/*!	\fn bool getTwist(uint8_t* array);
			\brief Getter of the twist package.
			
			Returns a boolean for the validity of twist.
			
			\param Array a buffer of integer.
			\return The boolean if twist of labmate is correct.
		*/bool 	getTwist(uint8_t* array);
		
		/*!	\fn bool isTwistValid();
			\brief Getter.
			
			Returns true if twist is correct.
			
			\return The boolean if twist of labmate is correct.
		*/bool	isTwistValid();
		
		/*!	\fn bool isOdometryValid();
			\brief Getter.
			
			Returns true if odometry is correct.
			
			\return The boolean if odometry of labmate is correct.
		*/bool	isOdometryValid();

		/*!	\fn LabmateProtocol *instance();
			\brief Static function.

			Returns a LabmateProtocol pointer.
			\sa LabmateProtocol()
		*/static 	LabmateProtocol* instance();
		
		/*! \fn ~LabmateProtocol();
			\brief Destroyer.
		*/	   ~LabmateProtocol (); 				
	
    private:
		/* ATTRIBUTES */

		/*! \var     uint8_t twist[3]
			\brief 	 Twist buffer of 3 bytes. 
		*/uint8_t 	 twist[3];
		
		/*! \var     uint8_t odometry[15]
			\brief 	 Odometry buffer of 15 bytes.
		*/uint8_t 	 odometry[15];
		
		/*! \var     uint8_t twistByte
			\brief 	 Index of twist byte.
		*/uint8_t	 twistByte;
		
		/*! \var     uint8_t odometryByte
			\brief 	 Index of odometry byte.
		*/uint8_t 	 odometryByte;

		static 	LabmateProtocol* singleton;		
		uint8_t conversionArray[8]; 
		uint8_t conversionArrayOdom[FLOAT_BITS];
		uint8_t rightDir;
		uint8_t leftDir;
		uint8_t rightPerc;
		uint8_t leftPerc;
		double 	omegaRight;
		double 	omegaLeft;
		uint8_t angularPerc;
		uint8_t linearPerc;
		double 	linearV;
		double 	angularV;
		float	x;
		float	y;
		float   theta;
		bool 	validTwist;
		bool	validOdometry;
		
		/* FUNCTIONS */
		
				LabmateProtocol();
		bool 	parseTwist();
		uint8_t*dec_bin(uint8_t dec);
		uint8_t bin_dec(uint8_t* array);
		
		uint8_t*dec_bin_Odo(float dec);
};

#endif  /* LABMATE_PROTOCOL_H */
