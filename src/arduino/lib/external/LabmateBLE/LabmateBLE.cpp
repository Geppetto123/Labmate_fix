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
#include "LabmateBLE.h"

LabmateBLE  	*LabmateBLE::singleton        	  = NULL;

/**************************************
 * Constructor
 **************************************/
LabmateBLE::LabmateBLE(){
	linearV=0.0;
	angularV=0.0;
}

/**************************************
 * Returns the istance of LabmateBLE
 **************************************/
LabmateBLE* LabmateBLE::instance() {
	if (NULL == singleton)
		singleton = new LabmateBLE();
	return singleton;
}

/***************************************************************
 * Conversion of the state value expressed in string to integer
 ***************************************************************/
int LabmateBLE::stato_corrente(char s[2]){
  int stato = 0;
  if(s[0]!='C' || s[1]!='C'){
    if(s[0]=='D'|| s[1]=='D')
       stato=1;
      else if(s[0]=='S'|| s[1]=='S')
       stato=2;
      else if(s[0]=='R'|| s[1]=='R')
       stato=3;
      else if(s[0]=='I'|| s[1]=='I')
       stato=4;
      else if(s[0]=='X'|| s[1]=='X')
       stato=5;
      else if(s[0]=='Y'|| s[1]=='Y')
       stato=6;
      else if(s[0]=='A'|| s[1]=='A')
       stato=7;
   }else
      stato=0;
   return stato;
}

/**********************************************************************************
 * Sets the angular and linear velocity of LabmateProtocol according to the state
 **********************************************************************************/
bool LabmateBLE::setCommand(int stato, LabmateProtocol   *protocol){
	switch (stato){
		case 1:{                                           // Case D - AVANTI
			
			linearV = protocol->getLinearV();
			angularV = protocol->getAngularV();
			linearV += INC_LIN;
			if(protocol->setTwist(linearV, angularV))return true;
			break;
		}
		
		case 2:{                                           // Case S - SINISTRA
			
			linearV = protocol->getLinearV();
			angularV = protocol->getAngularV();
			angularV += INC_ANG;
			if(protocol->setTwist(linearV, angularV))return true;
			break;
		}
		
		case 3:{                                           // Case R - RIGHT
			
			linearV = protocol->getLinearV();
			angularV = protocol->getAngularV();
			angularV -= INC_ANG;
			if(protocol->setTwist(linearV, angularV))return true; 
			break;
		}
		
		case 4:{                                           // Case I - INDIETRO
			
			linearV = protocol->getLinearV();
			angularV = protocol->getAngularV();
			linearV -= INC_LIN;
			if(protocol->setTwist(linearV, angularV))return true; 
			break;
		}
		
		case 5:{                                           // Case X - STOP ROTATE (V_ANG=0)
			
			linearV = protocol->getLinearV();
			angularV = protocol->getAngularV();
			angularV = 0 ;
			if(protocol->setTwist(linearV, angularV))return true; 
			break;
		}
		
		case 6:{                                           // Case Y - STOP GO (V_LIN=0)
			
			linearV = protocol->getLinearV();
			angularV = protocol->getAngularV();
			linearV = 0;
			if(protocol->setTwist(linearV, angularV))return true;
			break; 
		}
		
		case 7:{                                           // Case A - MEGA STOP (V_LIN=0 & V_ANG=0)
			
			linearV = protocol->getLinearV();
			angularV = protocol->getAngularV();
			linearV = 0 ;
			angularV = 0 ;
			if(protocol->setTwist(linearV, angularV)) return true;
			break;
		}
	}return false;
}

/***************************************************************************
 * Returns angular velocity
 ***************************************************************************/
double LabmateBLE::getAngularV(){
	return angularV;
}

/***************************************************************************
 * Returns linear velocity
 ***************************************************************************/
double LabmateBLE::getLinearV(){
	return linearV;
}