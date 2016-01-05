
#ifndef _DerWeg_ThreadSafeLogging_h_
#define _DerWeg_ThreadSafeLogging_h_

#include <iostream>
#include <boost/thread.hpp>

namespace DerWeg {

  static boost::mutex coutmutex;
  static boost::mutex cerrmutex;

  #define LOUT(x) { boost::unique_lock<boost::mutex> lock (DerWeg::coutmutex); std::cout << x; }
  #define EOUT(x) { boost::unique_lock<boost::mutex> lock (DerWeg::cerrmutex); std::cerr << x; }

}

#endif
