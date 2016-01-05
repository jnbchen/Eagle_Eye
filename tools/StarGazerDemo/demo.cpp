// File:           demo.cpp
// Creation Date:  Wednesday, December 16 2009
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#include <iostream>

#include "StarGazer.hpp"
#include <string>
#include <termios.h>
#include <fcntl.h>

using namespace std;

string binary (unsigned int i) {
   string s;
   for (unsigned int j=1; j<=16; j++) {
     if (i&0x01)
       s=string("1")+s;
     else
       s=string("0")+s;
     i=i>>1;
     if (j%4==0)
       s=string(" ")+s;
  }
  return s;
}

int main( int argc, char** argv)
{
  if (argc>=2 && (argv[1]==std::string("-h") || argv[1]==std::string("--help"))) {
    std::cerr << "StarGazer-Demo- und Testprogramm. Aufruf: \n";
    std::cerr << argv[0] << " [Device [Key1=Value1 Key2=Value2 ...]]\n";
    std::cerr << "  Device: Device, das mit StarGazer spricht (default: /dev/ANICAR_USB_2)\n";
    std::cerr << "  Key=Value: Parameter, die zu Programmstart an den StarGazer gesendet werden\n";
    return -1;
  }

  struct termios termattr, save_termattr;
  tcgetattr (STDIN_FILENO, &save_termattr);
  termattr=save_termattr;
  termattr.c_lflag&=~ICANON;
  termattr.c_lflag&=~ECHO;
  tcsetattr (STDIN_FILENO, TCSANOW, &termattr);
  fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
                                 
  try{
    StarGazer sg(argc>=2 ? argv[1] : "/dev/ANICAR_USB_2");
    for (int i=2; i<argc; ++i) {
      std::string key="";
      std::string value="";
      int j=0;
      bool is_key=true;
      char c=argv[i][j];
      while (c!='\0') {
        if (c=='=')
          is_key=false;
        else if (is_key)
          key+=c;
        else
          value+=c;
        ++j;
        c = argv[i][j];
      }
      if (key.length()>0 && value.length()>0) {
        cout << "setting " << key << "=" << value << endl;
        sg.write_parameter( key.c_str(), value.c_str() );
        sleep (1);
      }
    }

    std::cerr << "MarkMode \t"   << sg.read_parameter( "MarkMode" )  << "\n";
    std::cerr << "HeightFix: \t" << sg.read_parameter( "HeightFix" ) << "\n";
    std::cerr << "MarkType: \t"  << sg.read_parameter( "MarkType" )  << "\n";


    sg.start_calc();

    char command = ' ';
    while( command!='q' ) {
      StarGazer::PositionData pd;

      pd = sg.get_position();
      
      if( pd.dead )
      {
        cout << "dead!\n";
      }
      else
      {
        cout << "bin_id, x, y, z, theta, id: " << binary(pd.id) << ", " << pd.x << ", "  << pd.y << ", "  << pd.z << ", "  << pd.theta << ", " << pd.id << std::endl; 
      }
      ssize_t size = read (STDIN_FILENO, &command, 1);
      if (size==0) command=' ';
                      
    }
  }catch(std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    std::cerr << "Type '" << argv[0] << " --help' to see program options" << std::endl;
  }
  tcsetattr (STDIN_FILENO, TCSANOW, &save_termattr);
}
