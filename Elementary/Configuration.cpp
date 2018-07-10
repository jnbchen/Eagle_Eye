
#include <iostream>
#include "Configuration.h"
#include "PluginFactory.h"
#include "ThreadSafeLogging.h"

using namespace DerWeg;

Configuration* Configuration::instance = NULL;

Configuration::Configuration(): cr(NULL) {;}

Configuration::~Configuration() { clear (); }

void Configuration::clear () {
  while (module_name.size()>0)
    removeModule(module_name[0]);
};

Configuration* Configuration::getInstance() {
  if ( !instance )
    instance = new Configuration;
  return instance;
}

bool Configuration::init(const ConfigReader& cfg) {
  cr = &cfg;
  std::vector<std::string> which;

  cr->get ("Modules", which);
  for (unsigned int i=0; i<which.size(); ++i) {
    if (!addModule(which[i],false)) {
      EOUT("Could not create module " << which[i] << std::endl);
      return false;
    }
  }
  for (unsigned int i=0; i<which.size(); ++i) {
    if (!startModule(which[i])) {
      EOUT("Could not start module " << which[i] << std::endl);
      return false;
    }
  }
  return true;

}

bool Configuration::addModule (const std::string name, bool do_start) {
  boost::unique_lock<boost::mutex> lock (cfmutex);
  try{
    if (!cr) return false;  // no config reader assigned
    KogmoThread* ptr = DerWeg::PluginFactory<KogmoThread>::getPlugin (name);
    if (!ptr) return false;  // plugin of name 'name' does not exist
    ptr->init (*cr);
    if (do_start) {
      ptr->runThread ();
    }
    module_ptr.push_back (ptr);
    module_name.push_back (name);
    module_started.push_back (do_start);
    return true;
  }catch(std::exception& e) {
    EOUT("Exception in Configuration::addModule: " << e.what() << std::endl);
    return false;
  }
}

bool Configuration::startModule (const std::string name) {
  boost::unique_lock<boost::mutex> lock (cfmutex);
  for (unsigned int i=0; i<module_name.size(); ++i) {
    if (module_name[i]==name) {
      if (module_started[i]) return true;  // module already started
      try{
        module_ptr[i]->runThread ();
        module_started[i]=true;
        return true;
      }catch(std::exception& e) {
        EOUT("Exception in Configuration::startModule: " << e.what() << std::endl);
        return false;
      }
    }
  }
  return false;
}

bool Configuration::removeModule (const std::string name) {
  boost::unique_lock<boost::mutex> lock (cfmutex);
  for (unsigned int i=0; i<module_name.size(); ++i) {
    if (module_name[i]==name) {
      try{
        if (module_started[i]) {
          //EOUT("Stop Module " << name << " ... ");
          module_ptr[i]->stopThread();
          //EOUT("stopped\n");
        }
        module_ptr[i]->deinit();
        delete module_ptr[i];
        module_ptr.erase (module_ptr.begin()+i);
        module_name.erase (module_name.begin()+i);
        module_started.erase (module_started.begin()+i);
        return true;
      }catch(std::exception& e) {
        EOUT("Exception in Configuration::removeModule: " << e.what() << std::endl);
        return false;
      }
    }
  }
  return false;  // module with name 'name' not found
}

std::vector<std::string> Configuration::presentModules () {
  boost::unique_lock<boost::mutex> lock (cfmutex);
  return module_name;
}

std::vector<std::string> Configuration::allModules () {
  return DerWeg::PluginFactory<KogmoThread>::pluginList();
}
