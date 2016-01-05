
#ifndef _DerWeg_GigE_h_
#define _DerWeg_GigE_h_

#define _LINUX
#define _x86

#include <PvApi.h>
#include <boost/thread.hpp>

namespace DerWeg {

  /** \brief thread safe interface to access Pv functions. Cf. Prosilica API for documentation */ 
  class GigE {
    static unsigned int pv_initialized;  ///< number of times PvInitialize was called
    static boost::mutex pv_mutex;        ///< mutex to access Pv functions
    
  public:
    static tPvErr PvInitialize ();
    static void PvUnInitialize ();
    static unsigned long PvCameraCount ();
    static unsigned long PvCameraList (tPvCameraInfo*, unsigned long, unsigned long*);
    static tPvErr PvCameraOpen (unsigned long, tPvAccessFlags, tPvHandle*);
    static tPvErr PvCameraOpenByAddr (unsigned long, tPvAccessFlags, tPvHandle*);
    static tPvErr PvCaptureAdjustPacketSize (tPvHandle, unsigned long);
    static tPvErr PvCameraClose (tPvHandle);
    static tPvErr PvCommandRun (tPvHandle, const char*);
    static tPvErr PvAttrUint32Set (tPvHandle, const char*, tPvUint32);
    static tPvErr PvAttrUint32Get (tPvHandle, const char*, tPvUint32*);
    static tPvErr PvAttrRangeUint32 (tPvHandle, const char*, tPvUint32*, tPvUint32*);
    static tPvErr PvAttrFloat32Set (tPvHandle, const char*, tPvFloat32);
    static tPvErr PvAttrFloat32Get (tPvHandle, const char*, tPvFloat32*);
    static tPvErr PvAttrEnumSet (tPvHandle, const char*, const char*);
    static tPvErr PvCaptureQueueFrame (tPvHandle, tPvFrame*, tPvFrameCallback);
    static tPvErr PvCaptureStart (tPvHandle);
    static tPvErr PvCaptureEnd (tPvHandle);
    static tPvErr PvCaptureQueueClear (tPvHandle);
  };
  
}

#endif
