//to execute:
//./ConeDetection /PATHNAME/*rect.png





#include "ConeDetection.h"

int main (int argc, char** argv){
  ConeDetection ConeDetection1;
  ConeDetection1.init();
  ConeDetection1.execute(argc, argv);
  return 0;
}
