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

#include "MotorsDriver.h"

MotorsDriver    *MotorsDriver::singleton    = NULL;

DuePWM          *MotorsDriver::pwm          = NULL;

#ifdef TIMED_COMPUTATION
DueTimer         MotorsDriver::timer        = Timer2;
#endif

/**************************************
 * Returns an istance of MotorsDriver
 **************************************/
MotorsDriver *MotorsDriver::instance() {
	if (NULL == singleton)
		singleton = new MotorsDriver();
	return singleton;
}

/**************************************
 * Constructor of MotorsDriver
 **************************************/
MotorsDriver::MotorsDriver(){
	motors 	     = 0;
    encoders     = NULL;
	frequency1   = 0;
	frequency2   = 0;
    pidsArray[0] = NULL;
    pidsArray[1] = NULL;
    pidsArray[2] = NULL;
    pidsArray[3] = NULL;
    samplePeriod = 1000;

    pwm = new DuePWM(0,0);

#ifdef TIMED_COMPUTATION
	timer.attachInterrupt(computePID).setPeriod(1000000).start();
#endif
}

/********************************************************************************************
 * Generates numMotorsF1 + numMotorsF2 PWM signals with frequencies freq1, freq2 respectively
 ********************************************************************************************/
bool MotorsDriver::newMotors(uint8_t numMotorsF1, uint16_t freq1, uint8_t numMotorsF2, uint16_t freq2) {
	if ((numMotorsF1 == 0 && numMotorsF2 == 0) || ((freq1==0 && numMotorsF1>0) || (freq2==0 && numMotorsF2>0)) || motors + numMotorsF1 + numMotorsF2 > 4)
		return false;
	
	if (freq1 != frequency1 && (freq1 == frequency2 || frequency2==0)) {
		uint8_t	 numMotorsTemp = numMotorsF2;
		uint16_t freqTemp = freq2;
		
		freq2 = freq1;
		freq1 = freqTemp;
		numMotorsF2 = numMotorsF1;
		numMotorsF1 = numMotorsTemp;
	}
	
	if (freq2 != frequency2 && (freq2 == frequency1 || frequency1 == 0)) {
		uint8_t	 numMotorsTemp = numMotorsF1;
		uint16_t freqTemp = freq1;
		
		freq1 = freq2;
		freq2 = freqTemp;
		numMotorsF1 = numMotorsF2;
		numMotorsF2 = numMotorsTemp;
	}
	
	if (freq1 == freq2) {
		numMotorsF1 += numMotorsF2;
		numMotorsF2 = 0;
	} 
	
	if (frequency1 != 0 && frequency2 != 0 && ((freq1 != frequency1 && freq1!=0) || (freq2 != frequency2 && freq2!=0)))
		return false;
	
	if (numMotorsF1 > 0) {
		frequency1 = freq1;
		pwm->setFreq1(freq1);
		
		for(; numMotorsF1 > 0; numMotorsF1--) {
			pwm->pinFreq1(6 + motors);
			pwm->pinDuty(6 + motors, 0);
			motors++;
		}
	}
	
	if (numMotorsF2 > 0) {
		frequency2 = freq2;
		pwm->setFreq2(freq2);
		
		for(; numMotorsF2 > 0; numMotorsF2--) {
			pwm->pinFreq2(6 + motors);
			pwm->pinDuty(6 + motors, 0);
			motors++;
		}
	}
    return true;
}

/*****************************************************
 * Generates numMotors PWM signals with frequency freq 
 *****************************************************/
bool MotorsDriver::newMotors(uint8_t numMotors, uint16_t freq) {
	return newMotors(numMotors, freq, 0, 0);
}

/***************************
 * Creates a PID 
 ***************************/
bool MotorsDriver::newPID(
							uint8_t         motor,
							double          Kp,
							double          Ki,
							double          Kd,
#ifndef TIMED_COMPUTATION
							uint8_t         controlDir
#else
							uint8_t         controlDir,
							uint32_t        samplePeriod
#endif
						) {

	if (NULL == encoders || motor > motors-1)
		return false;
	
	if (!encoders->exists(motor) || (controlDir != DIRECT && controlDir != REVERSE))
        return false;
	
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return false;

#ifdef TIMED_COMPUTATION
    if (samplePeriod > 8388480)
        return false;

	if(this->samplePeriod > samplePeriod) {
        this->samplePeriod = samplePeriod;
		setComputationPeriod(samplePeriod*1000);    /* Period expressed in microseconds */
    }
#else
    this->samplePeriod = EXTERNAL_CALL_PERIOD;      /* Period expressed in milliseconds */
#endif

	PIDStruct *pidStruct = (PIDStruct*) calloc(1,sizeof(PIDStruct));

	pidsArray[motor]	 = pidStruct;

	pidStruct->pid = new PID(&(pidStruct->impRead), &(pidStruct->motorPWM), &(pidStruct->impSet), 0, 0, 0, DIRECT);
    pidStruct->pid->SetOutputLimits(0, 255);
	pidStruct->pid->SetControllerDirection(controlDir);
    pidStruct->pid->SetMode(AUTOMATIC);
    pidStruct->pid->SetSampleTime(this->samplePeriod);      
    pidStruct->pid->SetTunings(Kp, Ki, Kd);

	return true;
}

/***************************
 * Computation of PID
 ***************************/
void MotorsDriver::computePID() {
    uint8_t motorIndex = singleton->motors;

    if (0 == motorIndex)
        return;

    double          periodInSec = singleton->samplePeriod/1000.0;           /* Period in s */
    EncodersDriver *encoders    = singleton->encoders;
    PIDStruct      *pidStruct;

	do {
        motorIndex--;
        pidStruct = singleton->pidsArray[motorIndex];
        if (pidStruct) {                                                    /* if(NULL != pidStruct) */
            pidStruct->impRead = encoders->getStep(motorIndex);
            pidStruct->impRead = abs(pidStruct->impRead);

            /* Returns true if the PID has been computed */
            if (pidStruct->pid->Compute())
        		pwm->pinDuty(6 + motorIndex, (uint8_t) pidStruct->motorPWM);

            double numberOfHoles  = pidStruct->impRead/4.0;
            double holesPerRound  = encoders->getCPR(motorIndex)/encoders->getWheelTransmissionRatio(motorIndex);
            double degreesPerHole = 360.0/holesPerRound;

            pidStruct->angularV   = numberOfHoles*degreesPerHole/periodInSec;
	    }
    } while(motorIndex > 0);
}

/* SETTERS */

/***************************************************
 * Sets the driver of the encoders
 ***************************************************/
bool MotorsDriver::setEncodersDriver(EncodersDriver *encoders) {
    if (singleton->encoders)                                                /* if (NULL != encoders) */
        return false;

    uint8_t pidIndex = 0;

    do {
        if (NULL != pidsArray[pidIndex])
            return false;
        pidIndex++;
    } while(pidIndex < motors);

    singleton->encoders = encoders;

    return true;
}

bool MotorsDriver::setPIDGains(uint8_t pid, double Kp, double Ki, double Kd) {
	if (!pidsArray[pid])
        return false;

    pidsArray[pid]->pid->SetTunings(Kp, Ki, Kd);

    return true;
}

/**********************************************
 * Sets the speed of the specific motor
 **********************************************/
bool MotorsDriver::setSpeed(uint8_t motor, uint8_t speed) {
	if (motor > motors-1 || speed > 100)
		return false;
	pidsArray[motor]->impSet = (speed*MAX_IMPULSES_PER_PERIOD)/100;
	return true;
}

/**************************************************************
 * Sets the desired number of impulses with the motor in motion
 **************************************************************/
/*bool MotorsDriver::setImpulses(uint8_t motor,double impulses) {
	if (motor > motors-1)
		return false;
	pidsArray[motor]->impSet = impulses;
	return true;
}*/

#ifdef TIMED_COMPUTATION
/****************************************************
 * Sets the period of the timer (in microseconds)
 ****************************************************/
void MotorsDriver::setComputationPeriod(uint32_t period) {
	timer.setPeriod(period);
	timer.start();
}
#endif

/* END SETTERS */

/* GETTERS */

/***************************************************
 * Gets the angular velocity
 ***************************************************/
double MotorsDriver::getAngularV(uint8_t motor) {
	if (motor > motors-1 || !pidsArray[motor])
		return -1;
    return pidsArray[motor]->angularV;
}

#ifdef TIMED_COMPUTATION
/***************************************************
 * Gets the period of computation
 ***************************************************/
long MotorsDriver::getComputationPeriod() {
	return timer.getPeriod();
}
#endif

/* END GETTERS */
