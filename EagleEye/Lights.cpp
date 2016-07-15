#include "Lights.h"

#include "../Elementary/ThreadSafeLogging.h" //LOUT

//stream
#include <fstream>

/*
//or fprintf
#include <stdio.h>
#include <stdlib.h>
*/



namespace DerWeg{

// send message to Arduino by writing to serial port ttyACM4
void Lights::SerialWrite(char Msg){

	//either stream
	std::fstream outfile1;
	outfile1.open("/dev/ttyACM4",std::ios::out | std::ios::trunc );
	if (outfile1.is_open()){
		outfile1<<Msg;
		outfile1.close();
	}
	else {
		LOUT("ERROR"<<std::endl);
	}

	//or fprintf
	/* arduino=fopen("/dev/ttyACM4","w");
	if(arduino!=NULL){
		fprintf(arduino,"%c", Msg);
    }
	else {
		LOUT("ERROR"<<std::endl);     //LOUT
	}
	fclose(arduino);
	*/

  }


  void Lights::left_indicator_on(){
	  SerialWrite('L');
  }

  void Lights::right_indicator_on(){
	  SerialWrite('R');
  }

  void Lights::indicator_off(){
	  SerialWrite('O');
  }

  void Lights::hazard_lights_on(){
	  SerialWrite('H');
  }

  void Lights::brake_light_on(){
	  SerialWrite('B');
  }

  void Lights::brake_light_off(){
	  SerialWrite('b');
  }




}


