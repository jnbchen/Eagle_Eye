#include "Lights.h"

#include "../Elementary/ThreadSafeLogging.h" //LOUT

#include<math.h> //round

#include <fstream>




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
		LOUT("Lights: could not open serial port"<<std::endl);
	}


  }


  void Lights::left_indicator_on(){
	  SerialWrite('L');
  }

  void Lights::right_indicator_on(){
	  SerialWrite('R');
  }

  void Lights::indicator_off(){
	  SerialWrite('o');
  }

  void Lights::hazard_lights_on(){
	  SerialWrite('H');
  }

  void Lights::brake_light_on(int percentage){
      char c;
      if(percentage>0 && percentage <95){
        c=round(percentage*0.1)+48; // char '0'=48 ,  '1'=49 , ...
        SerialWrite(c);
      }
      else if(percentage>=95){
        SerialWrite('0');
      }
      else{
        SerialWrite('b');
      }
  }

  void Lights::brake_light_off(){
	  SerialWrite('b');
  }




}


