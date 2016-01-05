
#include "GigE.h"

unsigned int DerWeg::GigE::pv_initialized (0);
boost::mutex DerWeg::GigE::pv_mutex;

tPvErr DerWeg::GigE::PvInitialize () {
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex);
  if (pv_initialized>0) {
    ++pv_initialized;
    return ePvErrSuccess;
  } else {
    ++pv_initialized;
    tPvErr err = ::PvInitialize();
    if (err == ePvErrSuccess) {
      pv_initialized = true;
    }
    return err;
  }
}

void DerWeg::GigE::PvUnInitialize () {
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex);
  if (pv_initialized==1) {
    ::PvUnInitialize ();
    pv_initialized=0;
  } else if (pv_initialized>0) {
    --pv_initialized;
  }
}

unsigned long DerWeg::GigE::PvCameraCount () { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCameraCount ();
}

unsigned long DerWeg::GigE::PvCameraList (tPvCameraInfo* a, unsigned long b, unsigned long* c) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCameraList (a,b,c);
}

tPvErr DerWeg::GigE::PvCameraOpenByAddr (unsigned long a, tPvAccessFlags b, tPvHandle* c) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCameraOpenByAddr (a,b,c);
}

tPvErr DerWeg::GigE::PvCaptureAdjustPacketSize (tPvHandle a, unsigned long b) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCaptureAdjustPacketSize (a,b);
}

tPvErr DerWeg::GigE::PvCameraClose (tPvHandle a) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCameraClose (a);
}

tPvErr DerWeg::GigE::PvCommandRun (tPvHandle a, const char* b) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCommandRun (a,b);
}

tPvErr DerWeg::GigE::PvAttrUint32Set (tPvHandle a, const char* b, tPvUint32 c) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvAttrUint32Set (a,b,c);
}

tPvErr DerWeg::GigE::PvAttrRangeUint32 (tPvHandle a, const char* b, tPvUint32* c, tPvUint32* d) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvAttrRangeUint32 (a,b,c,d);
}

tPvErr DerWeg::GigE::PvAttrEnumSet (tPvHandle a, const char* b, const char* c) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvAttrEnumSet (a,b,c);
}

tPvErr DerWeg::GigE::PvCaptureQueueFrame (tPvHandle a, tPvFrame* b, tPvFrameCallback c) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCaptureQueueFrame (a,b,c);
}

tPvErr DerWeg::GigE::PvCaptureStart (tPvHandle a) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCaptureStart (a);
}

tPvErr DerWeg::GigE::PvCaptureEnd (tPvHandle a) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCaptureEnd (a);
}

tPvErr DerWeg::GigE::PvCaptureQueueClear (tPvHandle a) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCaptureQueueClear (a);
}

tPvErr DerWeg::GigE::PvCameraOpen (unsigned long a, tPvAccessFlags b, tPvHandle* c) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvCameraOpen (a,b,c);
}

tPvErr DerWeg::GigE::PvAttrUint32Get (tPvHandle a, const char* b, tPvUint32* c) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvAttrUint32Get (a,b,c);
}

tPvErr DerWeg::GigE::PvAttrFloat32Get (tPvHandle a, const char* b, tPvFloat32* c) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvAttrFloat32Get (a,b,c);
}

tPvErr DerWeg::GigE::PvAttrFloat32Set (tPvHandle a, const char* b, tPvFloat32 c) { 
  boost::unique_lock<boost::mutex> pv_lock (pv_mutex); 
  return ::PvAttrFloat32Set (a,b,c);
}
