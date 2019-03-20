/**********************************************************************************************
 * Lib:		Arduino MotorsDriver Library - Version 1.0.0
 * Date: 	7th December 2018
 *
 * by 		Veronica Nicla Cicchella 	<veronicanicla.cicchella@gmail.com>
 *			Roberto	Conca				<robertoconca1993@gmail.com>
 *			Marco Vitetta				<vito3200@hotmail.it>
 *
 *
 * This Library is licensed under a GPLv3 License
  *********************************************************************************************/
#ifndef MOTORS_DRIVER_H
#define MOTORS_DRIVER_H

#include <EncodersDriver.h>
#include <DuePWM.h>
#include <DueTimer.h>
#include <PID_v1.h>

/*! \def MAX_IMPULSES_PER_PERIOD
    \brief Maximum velocity of the motors expressed in impulses per 80 ms (period).
*/
#define MAX_IMPULSES_PER_PERIOD 8000
//#define TIMED_COMPUTATION                       /* Enables the timer which compute PIDs periodically. */

#ifndef TIMED_COMPUTATION
#define EXTERNAL_CALL_PERIOD    80                /* Expressed in milliseconds. */
#endif

/*! \typedef PIDStruct.
	\struct pidstruct.
	
	\param angularV
	\param impRead
	\param impSet
	\param impLast
	\param motorPWM
	\param *pid
	
    \brief  This struct contains the main components of PID. 
*/
typedef struct pidstruct {
			double           angularV;          /*!< Struct_Value impLast.       Angular velocity estimation expressed in °/s */
			double           impRead;           /*!< Struct_Value impRead.       Impulses read with the encoder.              */
			double           impSet;            /*!< Struct_Value impSet.        Impulses set.                                */
			double           motorPWM;			/*!< Struct_Value motorPWM.      Desired duty cycle of the PWM signal         */
			int32_t          impLast;           /*!< Struct_Value impLast.       Last impulses read                           */
			PID             *pid;				/*!< Struct_Value pid.        	 Pointer to the PID                           */
		} PIDStruct;

/*! \class MotorsDriver.
	\brief This class is responsible of implementing the Labmate Motors Driver.
*/
class MotorsDriver {
	
    /* ATTRIBUTES */
    private:

        /*! \var     MotorsDriver *singleton;
			\brief 	 Pointer to the instance of MotorsDriver class.
			\warning is a static var!
		*/
		static MotorsDriver *singleton;

		/*! \var     DuePWM *pwm
			\brief 	 Pointer to the DuePWM object which generates the PWM signals.
        */
		static DuePWM 		*pwm;

#ifdef TIMED_COMPUTATION

		/*! \var     DueTimer timer
			\brief 	 Timer used to compute periodically the PIDs (if TIMED_COMPUTATION is defined).
        */
		static DueTimer		 timer;
#endif

		/*! \var     EncodersDriver *encoders;
			\brief 	 Pointer to the EncodersDriver object which manages the encoders.
        */
        EncodersDriver      *encoders;

		/*! \var     PIDStruct *pidsArray[4];
			\brief 	 Array of the pointers to the PID structs.
        */
		PIDStruct 	        *pidsArray[4];

		/*! \var     uint8_t motors
			\brief 	 Counter of the motors managed.
        */
		uint8_t 			 motors;

		/*! \var     uint16_t frequency1
			\brief 	 Frequency of the first PWM signal.
        */
		uint16_t			 frequency1;

		/*! \var     uint16_t frequency2
			\brief 	 Frequency of the second PWM signal.
        */
		uint16_t			 frequency2;

		/*! \var     uint32_t samplePeriod
			\brief 	 Sampling period of the PIDs expressed in milliseconds.
        */
	    uint32_t             samplePeriod;
		
    /* FUNCTIONS */
    private:
							 MotorsDriver();

#ifdef TIMED_COMPUTATION
        static void          computePID(void);  /* Computation function is private  */
    public:                                     /* and static if timer is enabled.  */
    
        /*!	\fn      bool newPID(uint8_t motor, double Kp, double Ki, double Kd, uint8_t controlDir, uint32_t samplePeriod);
			\brief   Creates a new PID controller for the motor.
			
			\param   motor          Index of the motor that will be controlled by the PID.
			\param   Kp             Proportional gain of the PID.
			\param   Ki             Integral gain of the PID.
			\param   Kd             Derivative gain of the PID.
			\param   controlDir     Control direction of the PID (DIRECT or REVERSE).
			\param   samplePeriod   Computation period of the PID (if TIMED_COMPUTATION is defined).
			\return                 true if the PID has been created, false otherwise.
		*/
		bool 				 newPID(uint8_t motor, double Kp, double Ki, double Kd, uint8_t controlDir, uint32_t samplePeriod);

        /*!	\fn      setComputationPeriod(uint32_t period);
			\brief   Sets the period of the timer (if TIMED_COMPUTATION is defined) used to compute periodically the PIDs.
			
			\param   period The period of the timer expressed in microseconds.
		*/
		void 				 setComputationPeriod(uint32_t period);

		/*!	\fn      long getComputationPeriod();
			\brief   Returns the period of the timer (if TIMED_COMPUTATION is defined) used to compute periodically the PIDs.
			
			\return  The computation period of the timer.
        */
		long 				 getComputationPeriod();
#else
    public:
        /*! \fn      computePID();
            \brief   This is the function that computes PIDs.

                     The function gets the number of recognized impulses by the encoder,
                     computes the PID function and consequently sets the duty-cycle of
                     the PWM signal that controls the motor.
        */
        void                 computePID();      /* Computation function is public   */
                                                /* if timer isn't enabled.          */

        /*!	\fn      bool newPID(uint8_t motor, double Kp, double Ki, double Kd, uint8_t controlDir, uint32_t samplePeriod);
			\brief   Creates a new PID controller for the motor.
			
			\param   motor          Index of the motor that will be controlled by the PID.
			\param   Kp             Proportional gain of the PID.
			\param   Ki             Integral gain of the PID.
			\param   Kd             Derivative gain of the PID.
			\param   controlDir     Control direction of the PID (DIRECT or REVERSE).
			\return                 true if the PID has been created, false otherwise.
		*/
		bool 				 newPID(uint8_t motor, double Kp, double Ki, double Kd, uint8_t controlDir);
#endif

		/*! \fn      ~MotorsDriver();
			\brief   Destroyer.
		*/
							~MotorsDriver() {}

        /*!	\fn      MotorsDiver* instance();
			\brief 	 Returns the pointer to the instance of MotorsDriver class.
			\sa      MotorsDriver(),singleton()
		*/
		static MotorsDriver *instance();

        /*!	\fn      bool newMotors(uint8_t numMotors, uint16_t freq);
			\brief   Instantiates numMotors motors working at the frequency freq.

			\param   numMotors Number of motor to be controlled (up to 4 motors overall).
			\param   freq      Frequency (in Hz) of the PWM signal which controls the motors.
			\return            true if the motors have been created, false otherwise.
			\sa newMotors(uint8_t numMotorsF1, uint16_t freq1, uint8_t numMotorsF2, uint16_t freq2)
		*/
		bool 				 newMotors(uint8_t numMotors, uint16_t freq);

        /*!	\fn      bool newMotors(uint8_t numMotorsF1,uint16_t freq1,uint8_t numMotorsF2, uint16_t freq2);
			\brief   Instantiates numMotorsF1 and numMotorsF2 motors working at the frequency freq1 and freq2 respectevely.

			\param   numMotorsF1    Number of motor to be controlled with frequency freq1 (up to 4 motors overall).
			\param   freq1          Frequency (in Hz) of the PWM signal which controls the motors.
			\param   numMotorsF2    Number of motor to be controlled with frequency freq2 (up to 4 motors overall).
			\param   freq2          Frequency (in Hz) of the PWM signal which controls the motors.
			\return                 true if the motors have been created, false otherwise.
			\sa newMotors(uint8_t numMotorsF1, uint16_t freq1)
		*/
		bool 				 newMotors(uint8_t numMotorsF1,uint16_t freq1,uint8_t numMotorsF2, uint16_t freq2);

        /* SETTERS */

		/*!	\fn      bool setEncodersDriver(EncodersDriver *encoders);
			\brief   Sets the EncodersDriver object which manages the encoders.
			
			\param   encoders   Pointer to the EncodersDriver object.
			\return             true if the object gets set (if no PID was created previously), false otherwise.
        */
        bool                 setEncodersDriver(EncodersDriver *encoders);

        /*! \fn      bool setPIDGains(uint8_t pid, double Kp, double Ki, double Kd);
            \brief   Sets the gains of the PID.

            \param   pid    The index of the PID whose parameters will be set.
			\param   Kp     Proportional gain of the PID.
			\param   Ki     Integral gain of the PID.
			\param   Kd     Derivative gain of the PID.
        */
        bool                 setPIDGains(uint8_t pid, double Kp, double Ki, double Kd);

        /*!	\fn      bool setSpeed(uint8_t motor, uint8_t speed);
			\brief   Sets the speed (expressed in percentage) of the motor.
			
			\param   motor  The index of the motor whose speed will be set.
			\param   speed  The desired speed expressed in percentage.
			\return         true if the speed gets set, false otherwise.
		*/
		bool 				 setSpeed(uint8_t motor, uint8_t speed);
		//bool 				 setImpulses(uint8_t motor, double impulses);

        /* GETTERS */

        /*!	\fn      double getAngularV(uint8_t motor);
			\brief   Returns the rotation speed of the motor.
			
			\param   motor  The index of the motor whose speed will be returned.
			\return         The angular velocity of the motor.
		*/
		double 				 getAngularV(uint8_t motor);
};

#endif  /* MOTORS_DRIVER_H */
