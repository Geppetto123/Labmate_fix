#include "EncodersDriver.h"

//DueTimer         EncodersDriver::timer     = Timer1;
EncodersDriver  *EncodersDriver::singleton = NULL;

EncodersDriver::EncodersDriver(){
    encoders = MAX_ENCODERS;

    do {
        encoders--;
        encodersArray[encoders] = NULL;
    } while (encoders > 0);

    updatePeriod         = 10000;
    maxImpulsesPerPeriod = 10000 * MAX_IMPULSES_PER_US;
    //timer.attachInterrupt(update).setPeriod(4000);
}

EncodersDriver::~EncodersDriver(){
    while(encoders-- > 0) {
        delete encodersArray[encoders]->encoder;
        free(encodersArray[encoders]);
    }
    singleton = NULL;
}

/**************************************
 * Returns the istance of EncodersDriver
 **************************************/
EncodersDriver* EncodersDriver::instance() {
	if (NULL == singleton)
		singleton = new EncodersDriver();
	return singleton;
}

/**************************************
 * Instances a new Encoder
 **************************************/
bool EncodersDriver::newEncoder(uint8_t channelA, uint8_t channelB, uint32_t CPR, double wheelTransmissionRatio){
    if (MAX_ENCODERS == encoders)
        return false;

    EncoderStruct *encoderStruct          = (EncoderStruct*)calloc(1,sizeof(EncoderStruct));

    encodersArray[encoders]               = encoderStruct;
    encoderStruct->CPR                    = CPR;
    encoderStruct->encoder                = new Encoder(channelA, channelB);
    encoderStruct->encoder->write(0);
    encoderStruct->wheelTransmissionRatio = wheelTransmissionRatio;
    encoders++;
    //timer.start();

    return true;
}

bool EncodersDriver::newEncoder(uint8_t channelA, uint8_t channelB, uint32_t CPR){
    return newEncoder(channelA, channelB, CPR, 1);
}

/**********************************
 * Updates the status of the object
 **********************************/
int32_t EncodersDriver::update(uint8_t encoderIndex) {
    if (encoderIndex >= encoders || NULL == encodersArray[encoderIndex])
        return 0;

    EncoderStruct *encoderStruct = singleton->encodersArray[encoderIndex];
    int32_t        read          = encoderStruct->encoder->read();
    int32_t        lastRead      = encoderStruct->lastRead;

    // Overflow handling
    if (read > 0 && unsigned(INT_MAX - read) < singleton->maxImpulsesPerPeriod) {
        encoderStruct->encoder->write(0);
        read -= lastRead;
        lastRead = 0;
    } else
        // Underflow handling
        if (read < 0 && unsigned(read - INT_MIN) < singleton->maxImpulsesPerPeriod) {
            encoderStruct->encoder->write(0);
            read -= lastRead;
            lastRead = 0;
        }
    int32_t impulsesRead = read - lastRead;
    encoderStruct->step += impulsesRead;

    double   circumference    = 2*PI*WHEEL_RADIUS;
    uint32_t impulsesPerRound = 4*encoderStruct->CPR/encoderStruct->wheelTransmissionRatio;
        
    // Signals are quadrature encoded
    encoderStruct->distance  += impulsesRead*circumference/impulsesPerRound;
    encoderStruct->lastRead   = read;

    return impulsesRead;
}

bool EncodersDriver::exists(uint8_t encoderIndex) {
    if (encoderIndex < encoders && NULL != encodersArray[encoderIndex])
        return true;

    return false;
}

/* SETTERS */

/****************************************************
 * Set the period of the timer (in microseconds)
 ****************************************************/
bool EncodersDriver::setUpdatePeriod(uint32_t period) {
    if (0 == period)
        return false;

    updatePeriod         = period;
    maxImpulsesPerPeriod = period * MAX_IMPULSES_PER_US;
	//timer.setPeriod(period);
	//timer.start();
	
    return true;
}

/* END SETTERS */

/* GETTERS */

uint32_t EncodersDriver::getCPR(uint8_t encoderIndex) {
    if (encoderIndex > encoders-1)
        return 0;

    return encodersArray[encoderIndex]->CPR;
}
double EncodersDriver::getDistance(uint8_t encoderIndex) {
    if (encoderIndex > encoders-1)
        return 0;

    //noInterrupts();

    double distance = encodersArray[encoderIndex]->distance;

    encodersArray[encoderIndex]->distance = 0;

    //interrupts();

    return distance;
}

int32_t EncodersDriver::getStep(uint8_t encoderIndex) {
    if (encoderIndex > encoders-1)
        return 0;

    //noInterrupts();

    int32_t step = encodersArray[encoderIndex]->step;

    encodersArray[encoderIndex]->step = 0;

    //interrupts();

    return step;
}

double EncodersDriver::getWheelTransmissionRatio(uint8_t encoderIndex) {
    if (encoderIndex > encoders-1)
        return 0;

    return encodersArray[encoderIndex]->wheelTransmissionRatio;
}

/***************************************************
 * Gets the period of computation
 ***************************************************/
long EncodersDriver::getUpdatePeriod() {
	return updatePeriod;
}

long EncodersDriver::getEncoderRead(uint8_t encoderIndex) {
    if (encoderIndex > encoders-1)
        return 0;

    return encodersArray[encoderIndex]->encoder->read();
}

/* END GETTERS */
