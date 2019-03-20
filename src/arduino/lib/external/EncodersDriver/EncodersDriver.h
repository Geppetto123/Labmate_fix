#ifndef ENCODERS_DRIVER_H
#define ENCODERS_DRIVER_H

/* This optional setting causes Encoder to use more optimized code.
 * It must be defined before Encoder.h is included.                 */
#define ENCODERS_OPTIMIZE_INTERRUPTS

#include <Encoder.h>
#include <DueTimer.h>

#define INT_MAX				2147483647
#define INT_MIN 			-2147483648	
#define MAX_ENCODERS        4                           /* Maximum number of handled encoders    */
#define MAX_IMPULSES_PER_US 0.125                       /* Determined by the max speed of motors */
#define WHEEL_RADIUS        0.0725                      /* Expressed in meters                   */

typedef struct {
        double                 distance;
        double                 wheelTransmissionRatio;
        Encoder               *encoder;
        int32_t                lastRead;
        int32_t                step;
        uint32_t               CPR;                     /* Cycles (or counts) per revolution of the codewheel (or codestrip) */
} EncoderStruct;

class EncodersDriver {
	
    /* ATTRIBUTES */
    private:
        EncoderStruct         *encodersArray[MAX_ENCODERS];
        uint8_t                encoders;
        uint32_t               maxImpulsesPerPeriod;
        uint32_t               updatePeriod;
		static EncodersDriver *singleton;
		static DueTimer		   timer;
		
    /* FUNCTIONS */
    public:
							  ~EncodersDriver();
		static EncodersDriver *instance();
        bool                   exists(uint8_t encoderIndex);
        bool                   newEncoder(uint8_t channelA,
                                          uint8_t channelB,
                                          uint32_t CPR);
        bool                   newEncoder(uint8_t channelA,
                                          uint8_t channelB,
                                          uint32_t CPR,
                                          double wheelTransmissionRatio);

        /* SETTERS */
        bool                   setUpdatePeriod(uint32_t period);

        /* GETTERS */
        uint32_t               getCPR(uint8_t encoderIndex);
        double                 getDistance(uint8_t encoderIndex);
        int32_t                getStep(uint8_t encoderIndex);
        double                 getWheelTransmissionRatio(uint8_t encoderIndex);
		long 				   getUpdatePeriod();
		int32_t      	       update(uint8_t encoderIndex);
        long                   getEncoderRead(uint8_t encoderIndex);

    private:
							   EncodersDriver();
};

#endif  /* ENCODERS_DRIVER_H */
