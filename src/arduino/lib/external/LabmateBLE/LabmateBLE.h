/******************************************************************************************** // **
 * Lib:		Arduino LabmateBLE Library - Version 1.0.1
 * Date: 	23th November 2018
 *
 * by 		Veronica Nicla Cicchella 	<veronicanicla.cicchella@gmail.com>
 *			Roberto	Conca				<robertoconca1993@gmail.com>
 *			Marco Vitetta				<vito3200@hotmail.it>
 *
 *
 * This Library is licensed under a GPLv3 License
  **********************************************************************************************/
#ifndef LABMATE_BLE_H
#define LABMATE_BLE_H

#include <LabmateProtocol.h>
#include <string.h>	
#include <stdio.h>

/*! \def INC_LIN
    \brief Linear velocity rise.
   
    Increases the actual linear velocity of 10% respect the maximum linear velocity, expressed in m/s.
*/#define INC_LIN				0.113

/*! \def MINC_ANG
    \brief Angular velocity rise.
   
    Increases the actual angular velocity of 10% respect the maximum angular velocity, expressed in rad/s.
*/#define INC_ANG 			0.68


/*! \class LabmateBLE.
	\brief A Labmate Bluetooth Class.
  
    This class is responsible of implementing the functionally of Bluetooth comunication in Labmate.
*/
class LabmateBLE {

    private:
	
	/* PRIVATE ATTRIBUTES */
	
		static LabmateBLE 	*singleton;
		double 				linearV;
		double 				angularV;
		
	/* PRIVATE FUNCTIONS */
							LabmateBLE();
						   
    public:
	
	/* PUBLIC FUNCTIONS */
	
		/*! \fn ~LabmateBLE();
			\brief Destroyer.
								
			This is the distructor of LabmateBLE.
		*/						~LabmateBLE() {}
							   
		/*!	\fn LabmateBLE *instance();
			\brief Static function.

			Returns a LabmateBLE pointer.
			\sa LabmateBLE()
		*/static LabmateBLE		*instance();
		
		/*!	\fn int stato_corrente(char* command);
			\brief From command to state.

			Converts the buffer of char(command) into a state (0-7).
			
			\param command a char buffer.
			\return The create result.
		*/int                    stato_corrente(char* command);
		
		/*!	\fn bool setCommand(int stato, LabmateProtocol *protocol);
			\brief Setter of the velocity in LabmateProtocol.

			Takes a LabmateProtocol pointer and an integer and returns a boolean value.
			
			\param stato an integer.
			\param protocol an LabmateProtocol's pointer.
			
			\return Boolean if stato is in the range [1 7].
		*/bool					setCommand(int stato, LabmateProtocol   *protocol);
		
		/*!	\fn double getAngularV();
			\brief Getter.
			
			Returns an angular velocity.
			
			\return The angular velocity of command to Labmate.
		*/double 				getAngularV();
		
		/*!	\fn double getLinearV();
			\brief Getter.
			
			Returns a linear velocity.
			
			\return The linear velocity of command to Labmate.
		*/double 				getLinearV();
							   
};
#endif  /* LABMATE_BLE_H */