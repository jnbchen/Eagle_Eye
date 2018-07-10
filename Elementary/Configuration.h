
#ifndef _DerWeg_CONFIGURATION_H__
#define _DerWeg_CONFIGURATION_H__

#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include "../Elementary/ConfigReader.h"

#include "../Elementary/KogmoThread.h"

namespace DerWeg {

  class Configuration {
  private:
    static Configuration* instance;           ///< pointer onto the only instance of class Configuration (singleton)
    Configuration();                          ///< private constructor (since singleton)
    ~Configuration();                         ///< private destructor to avoid someone deleting Configuration

    boost::mutex cfmutex;                     ///< mutex to lock when accessing the configuration

    std::vector<KogmoThread*> module_ptr;
    std::vector<std::string> module_name;
    std::vector<bool> module_started;

    const ConfigReader* cr;

  public:
    static Configuration* getInstance();     ///< method to get the only instance of Configuration (singleton)

    /** add module with given name and eventually start it as thread */
    bool addModule (const std::string name, bool do_start =true);
    /** start a module as thread if not yet done before */
    bool startModule (const std::string name);
    /** stop and remove module with given name */
    bool removeModule (const std::string);
    std::vector<std::string> presentModules ();
    std::vector<std::string> allModules ();

    bool init (const ConfigReader&);         ///< start modules described. Return true on success
    void clear ();

  };

#define CONFIG DerWeg::Configuration::getInstance()    // macro to simplify accessing the configuration

} // namespace DerWeg

#endif // _DerWeg_CONFIGURATION_H__
