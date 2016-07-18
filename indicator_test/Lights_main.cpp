#include "Lights.h"
#include <iostream>

int main(){
	Lights lights1;
	char Msg;

	while(1){
		std::cin>>Msg;
		lights1.test_SerialWrite(Msg);
	}

	return 0;
}
