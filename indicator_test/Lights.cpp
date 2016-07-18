#include "Lights.h"
#include <iostream>

//stream
//#include <fstream>


//or fprintf
#include <stdio.h>
#include <stdlib.h> 




//namespace DerWeg{


void Lights::SerialWrite(char Msg){
	  
	//either stream
	/*std::fstream outfile1;
	outfile1.open("/dev/ttyACM3",std::ios::out | std::ios::trunc );
	if (outfile1.is_open()){
		outfile1<<Msg;
		outfile1.close();
	}
	else {
		std::cout<<"ERROR"<<std::endl;   //LOUT
	}*/
	
	//or fprintf
	FILE* arduino;
	arduino=fopen("/dev/ttyACM3","w");
	if(arduino!=NULL){
		fprintf(arduino,"%c", Msg);
    }
	else {
		std::cout<<"ERROR"<<std::endl;   //LOUT
	}
	fclose(arduino);
	

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
  
  void Lights::test_SerialWrite(char Msg){
	  SerialWrite(Msg);
  }
  
  
  
//}



