
#include <iostream>

#include "../Elementary/PluginFactory.h"
#include "../Elementary/ConfigReader.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Configuration.h"
#include <opencv/highgui.h>


/**
* \brief Hauptprogramm
* \param[in] argc number of arguments
* \param[in] argv argument list
* \return 0 - success, else - error
*/
int main(int argc, char** argv)
{
  try {
    // Konfigurationsdatei lesen
    std::string cfgfile = "default.cfg";
    if (argc>=2)
      cfgfile = argv[1];
    DerWeg::ConfigReader cr(0, '#', '=');
    if (!cr.append_from_file (cfgfile.c_str(), true))
      throw std::invalid_argument (std::string("could not read configuration file \"")+cfgfile+"\"");

    try{
      if (!DerWeg::Configuration::getInstance()->init (cr))  // initialize configuration and start modules
        throw std::invalid_argument ("Some modules could not be initialized.");

      EOUT ("OKAY: all modules started\n");
      while (!DerWeg::Blackboard::getInstance()->getExitProgram()) {  // wait until termination request
        cv::waitKey (100);  // needs to be executed in the main thread for some versions of highgui
//        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
//        EOUT (".");
      }
    }catch(std::exception& e){  // Exception hier fangen, um die Deinitialisierung noch durchlaufen zu koennen
      EOUT(e.what() << std::endl);
      DerWeg::Blackboard::getInstance()->setExitProgram();
    }

    DerWeg::Configuration::getInstance()->clear ();  // deinitialize modules
    LOUT(std::endl);
    return 0;
  } catch (std::exception& e) {
    EOUT(e.what() << std::endl);
    return -1;
  }

}
