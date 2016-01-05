
#include "KogmoThread.h"
#include <boost/bind.hpp>
#include <iostream>

DerWeg::KogmoThread::KogmoThread () : the_thread (NULL) {;}

DerWeg::KogmoThread::~KogmoThread () {
  if (the_thread)
    stopThread();
}

void DerWeg::KogmoThread::runThread () {
  if (the_thread)
    return;
  the_thread = new boost::thread(boost::bind(&DerWeg::KogmoThread::execute, this));
}

void DerWeg::KogmoThread::stopThread () {
  if (!the_thread)
    return;
  the_thread->interrupt();
  the_thread->join();
  delete the_thread;
  the_thread=NULL;
}
