
#define _LINUX
#define _x86

#include <iostream>
#include <algorithm>
#include "GigECameraDriver.h"
#include "../Elementary/ThreadSafeLogging.h"

using namespace DerWeg;

#define BUFFERS_ON_BUS 2

std::vector<GigECameraDriver*> DerWeg::GigECameraDriver::driversList;
boost::mutex DerWeg::GigECameraDriver::mutexDriversList;
std::vector<DerWeg::GigECameraDriver::ExtendedImageBuffer*> DerWeg::GigECameraDriver::lostBuffers;
boost::mutex DerWeg::GigECameraDriver::mutexLostBuffers;

GigECameraDriver::ExtendedImageBuffer::ExtendedImageBuffer () : locks (0), bufferRead (true) {
  resize (1,1,1);
  frame.ImageBuffer=image.data;
  frame.ImageBufferSize=image.step*image.rows;
  frame.AncillaryBuffer=NULL;
  frame.AncillaryBufferSize=0;
}


GigECameraDriver::ExtendedImageBuffer::ExtendedImageBuffer (unsigned int width, unsigned int height, int channels) : ImageBuffer (width,height,channels) {
  frame.ImageBuffer=image.data;
  frame.ImageBufferSize=image.step*image.rows;
  frame.AncillaryBuffer=NULL;
  frame.AncillaryBufferSize=0;
}


GigECameraDriver::ExtendedImageBuffer::ExtendedImageBuffer (const GigECameraDriver::ExtendedImageBuffer& b) : ImageBuffer(b.clone()), locks(b.locks), bufferRead(b.bufferRead) {
  frame.ImageBuffer=image.data;
  frame.ImageBufferSize=image.step*image.rows;
  frame.AncillaryBuffer=NULL;
  frame.AncillaryBufferSize=0;
}


void GigECameraDriver::ExtendedImageBuffer::resize (unsigned int width, unsigned int height, int channels) {
  image = cv::Mat (height, width, channels==1 ? CV_8UC1 : CV_8UC3);
  frame.ImageBuffer=image.data;
  frame.ImageBufferSize=image.step*image.rows;
  frame.AncillaryBuffer=NULL;
  frame.AncillaryBufferSize=0;
}

DerWeg::GigECameraDriver::GigECameraDriver (unsigned int n, unsigned int ip1) throw (std::invalid_argument) : ip(ip1), didStart(false), bufferOverflow(false), buffers(n+1+BUFFERS_ON_BUS) {
  boost::unique_lock<boost::mutex> lockDriversList (mutexDriversList);
  if (driversList.size()==0) {
    if (DerWeg::GigE::PvInitialize())
      throw std::invalid_argument ("PvInitialize() failed in DerWeg::GigECameraDriver::GigECameraDriver");
  }
  // search for cameras on bus
  unsigned int trialcount=0;
  unsigned int num_cameras=0;
  while((num_cameras=DerWeg::GigE::PvCameraCount())==0) {
    // if none was found, wait and retry (at most 10 times)
    if (++trialcount>10) {
      //throw std::invalid_argument ("could not find camera on bus in DerWeg::GigECameraDriver::GigECameraDriver");
      break;
    }
    usleep(100000);
  }
  // select the camera
  if (ip==0) {
    tPvCameraInfo camlist [1];
    DerWeg::GigE::PvCameraList(camlist, 1, NULL);
    ip = camlist[0].UniqueId;
  } else {
    ip=ip1;
  }
  // open camera
  if (DerWeg::GigE::PvCameraOpenByAddr(ip,ePvAccessMaster,&handle)!=ePvErrSuccess)
    throw std::invalid_argument ("unable to open camera in DerWeg::GigECameraDriver::GigECameraDriver");
  DerWeg::GigE::PvCaptureAdjustPacketSize(handle,8228);
  for (unsigned int i=0; i<buffers.size(); ++i)
    buffers[i]=NULL;
  setImageFormat (Mono8);
  // register driver at drivers list
  driversList.push_back (this);
}


DerWeg::GigECameraDriver::~GigECameraDriver () throw () {
  while (didStart)
    stopTransmission ();
  DerWeg::GigE::PvCameraClose(handle);
  boost::unique_lock<boost::mutex> lockDriversList (mutexDriversList);
  boost::unique_lock<boost::mutex> lockLostBuffers (mutexLostBuffers);
  boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
  for (unsigned int i=0; i<buffers.size(); ++i) {
    if (buffers[i]->locks==0)
      delete buffers[i];
    else
      lostBuffers.push_back(buffers[i]);
  }
  if (driversList.size()<=1)
    DerWeg::GigE::PvUnInitialize();
  // unregister driver at drivers list
  std::vector<GigECameraDriver*>::iterator it = std::find (driversList.begin(), driversList.end(), this);
  driversList.erase (it);
}


bool DerWeg::GigECameraDriver::setImageFormat (ColorMode mode) {
  try{
    unsigned int w=maxImageWidth();
    unsigned int h=maxImageHeight();
    return setImageFormat (w, h, 0, 0, mode);
  }catch(std::exception&) {
    return false;
  }
}

bool DerWeg::GigECameraDriver::setImageFormat (unsigned int w, unsigned int h, unsigned int u0, unsigned int v0, ColorMode mode) {
  boost::unique_lock<boost::mutex> lockLostBuffers (mutexLostBuffers);
  boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
  bool wasStarted=didStart;
  if (wasStarted) {
    // stop transmission
    DerWeg::GigE::PvCommandRun(handle,"AcquisitionStop");
    didStart=false;
  }
  bool okay=true;
  okay &= ((DerWeg::GigE::PvAttrUint32Set (handle, "Width", w))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (handle, "Height", h))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (handle, "RegionX", u0))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (handle, "RegionY", v0))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrEnumSet (handle, "PixelFormat", (mode==Mono8 ? "Mono8" : (mode==BGR24 ? "Bgr24" : "Bayer8"))))==ePvErrSuccess);

  // resize imagebuffers
  unsigned int channels = (mode==BGR24 ? 3 : 1);
  if (okay) {
    for (unsigned int i=0; i<buffers.size(); ++i) {
      if (buffers[i]==NULL) {
        // buffer does not yet exist -> create new buffer
        buffers[i]=new ExtendedImageBuffer (w, h, channels);
        buffers[i]->locks=0;
      } else if (buffers[i]->locks==0) {
        // buffer exists and is not in use -> resize
        buffers[i]->resize (w, h, channels);
        buffers[i]->locks=0;
      } else {
        // buffer exists and is in use -> move buffer to lostBuffers and create new buffer
        lostBuffers.push_back (buffers[i]);
        buffers[i]=new ExtendedImageBuffer (w, h, channels);
        buffers[i]->locks=0;
      }
    }
  }

  if (wasStarted) {
    // restart transmission
    bool okay1 = (DerWeg::GigE::PvCommandRun(handle,"AcquisitionStart")==ePvErrSuccess);
    okay=okay && okay1;
    bufferOverflow = false;
    didStart = okay1;
    putFrameOnBus(BUFFERS_ON_BUS);
  }

  return okay;
}


void DerWeg::GigECameraDriver::putFrameOnBus (unsigned int k) {
  if (!didStart || k==0)
    return;
  unsigned int iopt=buffers.size();
  for (unsigned int i=0; i<buffers.size(); ++i) {  // search oldest, non-locked buffer
    if (buffers[i]->locks==0 && (iopt>=buffers.size() || buffers[i]->capturetime<buffers[iopt]->capturetime)) {
      iopt=i;
    }
  }
  if (iopt<buffers.size()) {  // if buffer available, lock it and put it on the bus
    if (DerWeg::GigE::PvCaptureQueueFrame(handle,&(buffers[iopt]->frame),DerWeg::frameDoneCallback)!=ePvErrSuccess) {
      bufferOverflow=true;
    } else {
      buffers[iopt]->locks=-1;
    }
  } else {
    bufferOverflow=true;
  }
  putFrameOnBus (k-1);
}

void ::DerWeg::frameDoneCallback (tPvFrame* frame) {
  // search in the drivers list for appropriate driver object and frame bufferlist
  boost::unique_lock<boost::mutex> lockDriversList (GigECameraDriver::mutexDriversList);
  for (unsigned int i=0; i<GigECameraDriver::driversList.size(); ++i) {
    GigECameraDriver* that = GigECameraDriver::driversList[i];
    boost::unique_lock<boost::mutex> lockBuffers (that->mutexBuffers);
    for (unsigned int j=0; j<GigECameraDriver::driversList[i]->buffers.size(); ++j) {
      if (&(that->buffers[j]->frame)==frame) {
        that->buffers[j]->locks=0;
        if (frame->Status==ePvErrSuccess) {
          that->buffers[j]->bufferRead=false;
          that->buffers[j]->capturetime.update();
          that->conditionalBufferReleased.notify_all();
          that->numImages++;
        } else {
          // an error occurred during capturing. Ignore image
          that->buffers[j]->bufferRead=true;
          //EOUT("Dropped frame: "<<frame->Status << "\n");
        }
        if (that->didStart) {
          that->putFrameOnBus();
        }
        return;
      }
    }
  }
  // frame not found in drivers. search frame in lostBuffers
  boost::unique_lock<boost::mutex> lockLostBuffers (GigECameraDriver::mutexLostBuffers);
  for (unsigned int i=0; i<GigECameraDriver::lostBuffers.size(); ++i) {
    if (&(GigECameraDriver::lostBuffers[i]->frame)==frame) {
      delete GigECameraDriver::lostBuffers[i];
      GigECameraDriver::lostBuffers.erase (GigECameraDriver::lostBuffers.begin()+i);
      return;
    }
  }
}

const ImageBuffer* DerWeg::GigECameraDriver::getImage (bool blocking) {
  boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
  unsigned int iopt=buffers.size();
  while (iopt>=buffers.size()) {
    if (!didStart)
      return NULL;
    for (unsigned int i=0; i<buffers.size(); ++i) {  // search newest buffer
      if (buffers[i]->locks>=0 && (iopt>=buffers.size() || buffers[i]->capturetime>buffers[iopt]->capturetime)) {
        iopt=i;
      }
    }
    if (iopt>=buffers.size()) {
      return NULL;
    }
    if (blocking && buffers[iopt]->bufferRead) {
      // if blocking and buffer already read, wait for next buffer
      iopt=buffers.size();
      conditionalBufferReleased.wait(lockBuffers);
    }
  }
  buffers[iopt]->locks++;
  buffers[iopt]->bufferRead=true;
  return buffers[iopt];
}


void DerWeg::GigECameraDriver::releaseImage (const ImageBuffer* buf) {
  // search for buf in all drivers and all buffers
  boost::unique_lock<boost::mutex> lockDriversList (GigECameraDriver::mutexDriversList);
  bool buffer_found=false;
  for (unsigned int i=0; i<GigECameraDriver::driversList.size() && !buffer_found; ++i) {
    GigECameraDriver* that = driversList[i];
    boost::unique_lock<boost::mutex> lockBuffers (that->mutexBuffers);
    for (unsigned int j=0; j<that->buffers.size() && !buffer_found; ++j) {
      if (that->buffers[j]==buf) {
        that->buffers[j]->locks--;
        if (that->bufferOverflow && that->buffers[j]->locks==0) {
          // in case of buffer overflow immediately put buffer on bus
          that->buffers[j]->locks=-1;
          DerWeg::GigE::PvCaptureQueueFrame(that->handle,&(that->buffers[j]->frame),::DerWeg::frameDoneCallback);
          that->bufferOverflow=false;
        }
        buffer_found=true;
      }
    }
  }
  if (!buffer_found) {
    boost::unique_lock<boost::mutex> lockLostBuffers (GigECameraDriver::mutexLostBuffers);
    // buf not found, search in lostBuffers and remove buffer if found
    for (unsigned int i=0; i<GigECameraDriver::lostBuffers.size(); ++i) {
      if (GigECameraDriver::lostBuffers[i]==buf) {
        delete GigECameraDriver::lostBuffers[i];
        GigECameraDriver::lostBuffers.erase (GigECameraDriver::lostBuffers.begin()+i);
        break;
      }
    }
  }
}


bool DerWeg::GigECameraDriver::startTransmission () throw() {
  boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
  bool okay = true;
  okay &= (DerWeg::GigE::PvCaptureStart(handle)==ePvErrSuccess);
  okay &= (DerWeg::GigE::PvCommandRun(handle,"AcquisitionStart")==ePvErrSuccess);
  if (!okay) {
    DerWeg::GigE::PvCaptureEnd(handle);
  }
  bufferOverflow = false;
  didStart=okay;
  numImages=0;
  captureStart.update();
  putFrameOnBus(BUFFERS_ON_BUS);
  return okay;
}


bool DerWeg::GigECameraDriver::stopTransmission () throw() {
  bool okay=true;
  {
    boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
    didStart=false;
    conditionalBufferReleased.notify_all();
    DerWeg::GigE::PvCommandRun(handle,"AcquisitionStop");
    okay = (DerWeg::GigE::PvCaptureEnd(handle)==ePvErrSuccess);
  }
  DerWeg::GigE::PvCaptureQueueClear(handle);
  return okay;
}


double DerWeg::GigECameraDriver::frameRate () throw() {
  boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
  return 1000*static_cast<double>(numImages)/static_cast<double>(captureStart.elapsed_msec());
}


unsigned int DerWeg::GigECameraDriver::maxImageWidth() throw (std::invalid_argument) {
  tPvUint32 min, max;
  if (DerWeg::GigE::PvAttrRangeUint32 (handle, "Width", &min, &max)!=ePvErrSuccess)
    throw std::invalid_argument ("error calling PvAttrRangeUint32 in DerWeg::GigECameraDriver::maxImageWidth");
  return max;
}


unsigned int DerWeg::GigECameraDriver::maxImageHeight() throw (std::invalid_argument) {
  tPvUint32 min, max;
  if (DerWeg::GigE::PvAttrRangeUint32 (handle, "Height", &min, &max)!=ePvErrSuccess)
    throw std::invalid_argument ("error calling PvAttrRangeUint32 in DerWeg::GigECameraDriver::maxImageHeight");
  return max;
}


bool DerWeg::GigECameraDriver::setWhiteBalance (unsigned int u, unsigned int v) throw() {
  return (
  (DerWeg::GigE::PvAttrEnumSet(handle,"WhitebalMode","Manual")==ePvErrSuccess) &&
  (DerWeg::GigE::PvAttrUint32Set(handle,"WhitebalRed",u)==ePvErrSuccess) &&
  (DerWeg::GigE::PvAttrUint32Set(handle,"WhitebalBlue",v)==ePvErrSuccess) );
}


bool DerWeg::GigECameraDriver::setAutoWhiteBalance () throw() {
  return (DerWeg::GigE::PvAttrEnumSet(handle,"WhitebalMode","Auto")==ePvErrSuccess);
}


bool DerWeg::GigECameraDriver::setGain (unsigned int g) throw() {
  return (DerWeg::GigE::PvAttrUint32Set(handle,"GainValue",g)==ePvErrSuccess);
}


bool DerWeg::GigECameraDriver::setExposure (unsigned int e) throw() {
  return (
  (DerWeg::GigE::PvAttrEnumSet(handle,"ExposureMode","Manual")==ePvErrSuccess) &&
  (DerWeg::GigE::PvAttrUint32Set(handle,"ExposureValue",e)==ePvErrSuccess) );
}


bool DerWeg::GigECameraDriver::setAutoExposure (unsigned int e) throw() {
  return (
  (DerWeg::GigE::PvAttrEnumSet(handle,"ExposureMode","Auto")==ePvErrSuccess) &&
  (DerWeg::GigE::PvAttrUint32Set(handle,"ExposureAutoTarget",e)==ePvErrSuccess) );
}
