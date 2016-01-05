
#define _LINUX
#define _x86

#include <iostream>
#include <cstdio>
#include <algorithm>
#include "GigEStereoCameraDriver.h"
#include "../Elementary/ThreadSafeLogging.h"

using namespace DerWeg;

#define BUFFERS_ON_BUS 2

std::vector<GigEStereoCameraDriver*> DerWeg::GigEStereoCameraDriver::driversList;
boost::mutex DerWeg::GigEStereoCameraDriver::mutexDriversList;
std::vector<DerWeg::GigEStereoCameraDriver::StereoExtendedImageBuffer*> DerWeg::GigEStereoCameraDriver::lostBuffers;
boost::mutex DerWeg::GigEStereoCameraDriver::mutexLostBuffers;

GigEStereoCameraDriver::StereoExtendedImageBuffer::StereoExtendedImageBuffer () : locks (0), bufferRead (true) {
  resize (1,1,1);
  frameLeft.ImageBuffer=image.data;
  frameLeft.ImageBufferSize=image.step*image.rows;
  frameRight.ImageBuffer = image_right.data;
  frameRight.ImageBufferSize=image_right.step*image_right.rows;
  frameLeft.AncillaryBuffer=NULL;
  frameLeft.AncillaryBufferSize=0;
  frameRight.AncillaryBuffer=NULL;
  frameRight.AncillaryBufferSize=0;
  leftComplete=rightComplete=false;
}


GigEStereoCameraDriver::StereoExtendedImageBuffer::StereoExtendedImageBuffer (unsigned int width, unsigned int height, int channels) : ImageBuffer (width,height,channels,true) {
  bufferRead = true;
  frameLeft.ImageBuffer=image.data;
  frameLeft.ImageBufferSize=image.step*image.rows;
  frameRight.ImageBuffer = image_right.data;
  frameRight.ImageBufferSize=image_right.step*image_right.rows;
  frameLeft.AncillaryBuffer=NULL;
  frameLeft.AncillaryBufferSize=0;
  frameRight.AncillaryBuffer=NULL;
  frameRight.AncillaryBufferSize=0;
  leftComplete=rightComplete=false;
}


GigEStereoCameraDriver::StereoExtendedImageBuffer::StereoExtendedImageBuffer (const GigEStereoCameraDriver::StereoExtendedImageBuffer& b) : ImageBuffer(b.clone()), locks(b.locks), bufferRead(b.bufferRead) {
  frameLeft.ImageBuffer=image.data;
  frameLeft.ImageBufferSize=image.step*image.rows;
  frameRight.ImageBuffer = image_right.data;
  frameRight.ImageBufferSize=image_right.step*image_right.rows;
  frameLeft.AncillaryBuffer=NULL;
  frameLeft.AncillaryBufferSize=0;
  frameRight.AncillaryBuffer=NULL;
  frameRight.AncillaryBufferSize=0;
  leftComplete=b.leftComplete;
  rightComplete=b.rightComplete;
  leftCapture=b.leftCapture;
  rightCapture=b.rightCapture;
}


void GigEStereoCameraDriver::StereoExtendedImageBuffer::resize (unsigned int width, unsigned int height, int channels) {
  image = cv::Mat (height, width, channels==1 ? CV_8UC1 : CV_8UC3);
  image_right = image.clone();
  bufferRead = true;
  leftComplete = rightComplete = false;
  frameLeft.ImageBuffer=image.data;
  frameLeft.ImageBufferSize=image.step*image.rows;
  frameRight.ImageBuffer = image_right.data;
  frameRight.ImageBufferSize=image_right.step*image_right.rows;
  frameLeft.AncillaryBuffer=NULL;
  frameLeft.AncillaryBufferSize=0;
  frameRight.AncillaryBuffer=NULL;
  frameRight.AncillaryBufferSize=0;
}

DerWeg::GigEStereoCameraDriver::GigEStereoCameraDriver (unsigned int n) throw (std::invalid_argument) :
                                        didStart(false), bufferOverflow(false), buffers(n+2) {
    init(0,0);
}

DerWeg::GigEStereoCameraDriver::GigEStereoCameraDriver (unsigned int n, unsigned int leftIp, unsigned int rightIp)  throw (std::invalid_argument) :
                                        didStart(false), bufferOverflow(false), buffers(n+1+BUFFERS_ON_BUS) {
    init(leftIp, rightIp);
}

void DerWeg::GigEStereoCameraDriver::init(unsigned int leftIp, unsigned int rightIp) throw (std::invalid_argument) {

  boost::unique_lock<boost::mutex> lockDriversList (mutexDriversList);
  if (driversList.size()==0) {
    if (DerWeg::GigE::PvInitialize())
      throw std::invalid_argument ("PvInitialize() failed in DerWeg::GigEStereoCameraDriver::GigEStereoCameraDriver");
  }

  // select the camera
  if (leftIp == 0 && rightIp == 0) {
       unsigned int trialcount=0;
       unsigned int num_cameras=0;
       // search for cameras on bus
       while((num_cameras=DerWeg::GigE::PvCameraCount())<=2) {
         // if none was found, wait and retry (at most 10 times)
         if (++trialcount>10) {
           throw std::invalid_argument ("could not find stereo cameras on bus in DerWeg::GigEStereoCameraDriver::GigEStereoCameraDriver");
         }
         usleep(100000);
      }
      tPvCameraInfo camlist [2];
      DerWeg::GigE::PvCameraList(camlist, 2, NULL);
      unsigned int leftId = camlist[0].UniqueId;
      unsigned int rightId = camlist[1].UniqueId;
      // open camera
      if (DerWeg::GigE::PvCameraOpen(leftId,ePvAccessMaster,&leftHandle)!=ePvErrSuccess)
        throw std::invalid_argument ("unable to open left camera in DerWeg::GigEStereoCameraDriver::GigEStereoCameraDriver");
      if (DerWeg::GigE::PvCameraOpen(rightId,ePvAccessMaster,&rightHandle)!=ePvErrSuccess)
        throw std::invalid_argument ("unable to open right camera in DerWeg::GigEStereoCameraDriver::GigEStereoCameraDriver");
  } else {
      // open camera
      if (DerWeg::GigE::PvCameraOpenByAddr(leftIp,ePvAccessMaster,&leftHandle)!=ePvErrSuccess)
        throw std::invalid_argument ("unable to open left camera in DerWeg::GigEStereoCameraDriver::GigEStereoCameraDriver");
      if (DerWeg::GigE::PvCameraOpenByAddr(rightIp,ePvAccessMaster,&rightHandle)!=ePvErrSuccess)
        throw std::invalid_argument ("unable to open right camera in DerWeg::GigEStereoCameraDriver::GigEStereoCameraDriver");
  }
  DerWeg::GigE::PvAttrUint32Set(leftHandle, "StreamBytesPerSecond", 29239830);
  DerWeg::GigE::PvAttrUint32Set(rightHandle, "StreamBytesPerSecond", 29239830);
  DerWeg::GigE::PvAttrEnumSet(leftHandle, "FrameStartTriggerMode", "SyncIn2");
  DerWeg::GigE::PvAttrEnumSet(rightHandle, "FrameStartTriggerMode", "SyncIn2");
  DerWeg::GigE::PvCaptureAdjustPacketSize(leftHandle,1500);
  DerWeg::GigE::PvCaptureAdjustPacketSize(rightHandle,1500);
  for (unsigned int i=0; i<buffers.size(); ++i)
    buffers[i]=NULL;
  setImageFormat (Mono8);
  // register driver at drivers list

  driversList.push_back (this);
}

DerWeg::GigEStereoCameraDriver::~GigEStereoCameraDriver () throw () {
  while (didStart)
    stopTransmission ();
  DerWeg::GigE::PvCameraClose(leftHandle);
  DerWeg::GigE::PvCameraClose(rightHandle);
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
  std::vector<GigEStereoCameraDriver*>::iterator it = std::find (driversList.begin(), driversList.end(), this);
  driversList.erase (it);
}


bool DerWeg::GigEStereoCameraDriver::setImageFormat (ColorMode mode) {
  try{
    unsigned int w=maxImageWidth();
    unsigned int h=maxImageHeight();
    return setImageFormat (w, h, 0, 0, mode);
  }catch(std::exception&) {
    return false;
  }
}

bool DerWeg::GigEStereoCameraDriver::setImageFormat (unsigned int w, unsigned int h, unsigned int u0, unsigned int v0, ColorMode mode) {
  boost::unique_lock<boost::mutex> lockLostBuffers (mutexLostBuffers);
  boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
  bool wasStarted=didStart;
  if (wasStarted) {
    // stop transmission
    DerWeg::GigE::PvCommandRun(leftHandle,"AcquisitionStop");
    DerWeg::GigE::PvCommandRun(rightHandle,"AcquisitionStop");
    didStart=false;
  }
  bool okay=true;
  okay &= ((DerWeg::GigE::PvAttrUint32Set (leftHandle, "Width", w))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (leftHandle, "Height", h))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (leftHandle, "RegionX", u0))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (leftHandle, "RegionY", v0))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrEnumSet (leftHandle, "PixelFormat", (mode==Mono8 ? "Mono8" : (mode==BGR24 ? "Bgr24" : "Bayer8"))))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (rightHandle, "Width", w))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (rightHandle, "Height", h))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (rightHandle, "RegionX", u0))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrUint32Set (rightHandle, "RegionY", v0))==ePvErrSuccess);
  okay &= ((DerWeg::GigE::PvAttrEnumSet (rightHandle, "PixelFormat", (mode==Mono8 ? "Mono8" : (mode==BGR24 ? "Bgr24" : "Bayer8"))))==ePvErrSuccess);
  // resize imagebuffers
  unsigned int channels = (mode==BGR24 ? 3 : 1);
  if (okay) {
    for (unsigned int i=0; i<buffers.size(); ++i) {
      if (buffers[i]==NULL) {
        // buffer does not yet exist -> create new buffer
        buffers[i]=new StereoExtendedImageBuffer (w, h, channels);
        buffers[i]->locks=0;
      } else if (buffers[i]->locks==0) {
        // buffer exists and is not in use -> resize
        buffers[i]->resize (w, h, channels);
        buffers[i]->locks=0;
      } else {
        // buffer exists and is in use -> move buffer to lostBuffers and create new buffer
        lostBuffers.push_back (buffers[i]);
        buffers[i]=new StereoExtendedImageBuffer (w, h, channels);
        buffers[i]->locks=0;
      }
    }
  }

  if (wasStarted) {
    // restart transmission
    bool okay1 = (DerWeg::GigE::PvCommandRun(leftHandle,"AcquisitionStart")==ePvErrSuccess);
    okay1 &= (DerWeg::GigE::PvCommandRun(rightHandle,"AcquisitionStart")==ePvErrSuccess);
    okay=okay && okay1;
    bufferOverflow = false;
    didStart = okay1;
    putFrameOnBus(BUFFERS_ON_BUS);
  }

  return okay;
}

void printStats(tPvHandle handle) {
    tPvUint32 fdr= 0, fcr= 0, prr= 0, plr= 0;
    tPvFloat32 frr= 0;
    tPvErr err;
    err = DerWeg::GigE::PvAttrUint32Get(handle,"StatFramesCompleted",&fcr);
    err = DerWeg::GigE::PvAttrFloat32Get(handle,"StatFrameRate",&frr);
    err = DerWeg::GigE::PvAttrUint32Get(handle,"StatFramesDropped",&fdr);
    err = DerWeg::GigE::PvAttrUint32Get(handle,"StatPacketsReceived",&prr);
    err = DerWeg::GigE::PvAttrUint32Get(handle,"StatPacketsMissed",&plr);

    printf("Camera:\nFramerate: %f\nFrames dropped: %u \
            \nFrames completed: %u\nPackets received: %u\nPackets lost: %u\n\n-----------------------------------\n",
            frr, static_cast<unsigned int>(fdr), static_cast<unsigned int>(fcr), static_cast<unsigned int>(prr), static_cast<unsigned int>(plr));
	//return err;
}

void DerWeg::GigEStereoCameraDriver::putFrameOnBus (unsigned int k) {
  if (!didStart || k==0)
    return;
  unsigned int iopt=buffers.size();
  for (unsigned int i=0; i<buffers.size(); ++i) {  // search oldest, non-locked buffer
    if (buffers[i]->locks==0 && (iopt>=buffers.size() || buffers[i]->capturetime<buffers[iopt]->capturetime)) {
      iopt=i;
    }
  }
  if (iopt<buffers.size()) {  // if buffer available, lock it and put it on the bus
    if (DerWeg::GigE::PvCaptureQueueFrame(leftHandle,&(buffers[iopt]->frameLeft),DerWeg::stereoFrameDoneCallback)!=ePvErrSuccess ||
        DerWeg::GigE::PvCaptureQueueFrame(rightHandle,&(buffers[iopt]->frameRight),DerWeg::stereoFrameDoneCallback)!=ePvErrSuccess) {
      bufferOverflow=true;
    } else {
      buffers[iopt]->locks=-1;
      buffers[iopt]->leftComplete = false;
      buffers[iopt]->rightComplete = false;
      buffers[iopt]->capturetime.update();
    }
  } else {
    bufferOverflow=true;
  }
  putFrameOnBus (k-1);
}


void ::DerWeg::stereoFrameDoneCallback (tPvFrame* frame) {
  // search in the drivers list for appropriate driver object and frame bufferlist
  boost::unique_lock<boost::mutex> lockDriversList (GigEStereoCameraDriver::mutexDriversList);
/*  std::string bufferstate;
  for (unsigned int i=0; i<GigEStereoCameraDriver::driversList.size(); ++i) {
    boost::unique_lock<boost::mutex> lockBuffers (GigEStereoCameraDriver::driversList[i]->mutexBuffers);
    for (unsigned int j=0; j<GigEStereoCameraDriver::driversList[i]->buffers.size(); ++j) {
      const DerWeg::GigEStereoCameraDriver::StereoExtendedImageBuffer& bf (*GigEStereoCameraDriver::driversList[i]->buffers[j]);
      if (bf.locks>0) {
        bufferstate+='0'+bf.locks;
      } else if (bf.locks==0) {
        if (bf.bufferRead) {
          bufferstate+='-';
        } else {
          bufferstate+='+';
        }
      } else {
        if (bf.leftComplete) {
          if (bf.rightComplete) {
            bufferstate+='C';
          } else {
            bufferstate+='L';
          }
        } else if (bf.rightComplete) {
          bufferstate+='R';
        } else {
          bufferstate+='W';
        }
      }
    }    
    bufferstate+=' ';
  }  
  EOUT("\nBufferstate=" << bufferstate << " ");*/

  bool buffer_found=false;
  for (unsigned int i=0; i<GigEStereoCameraDriver::driversList.size(); ++i) {
    GigEStereoCameraDriver* that = GigEStereoCameraDriver::driversList[i];
    boost::unique_lock<boost::mutex> lockBuffers (that->mutexBuffers);
    // search buffer in buffer list
    unsigned int j=0;
    for ( ; j<that->buffers.size(); ++j) {
      if (&(that->buffers[j]->frameLeft)==frame) { // result from left camera
        that->buffers[j]->leftComplete = true;
        that->buffers[j]->leftCapture.update();
//        EOUT ("__Receive-buffer-" << j << "L__");
        buffer_found=true;
        break;
      } else if (&(that->buffers[j]->frameRight)==frame) {
        that->buffers[j]->rightComplete = true;
        that->buffers[j]->rightCapture.update();
//        EOUT ("__Receive-buffer-" << j << "R__");
        buffer_found=true;
        break;
      }
    }

    if (buffer_found) {
      // check whether both, left and right buffer are available
      if (that->buffers[j]->rightComplete && that->buffers[j]->leftComplete) {
        that->buffers[j]->locks=0;
        if (that->buffers[j]->frameLeft.Status==ePvErrSuccess && that->buffers[j]->frameRight.Status==ePvErrSuccess) {
          that->buffers[j]->bufferRead=false;
          that->buffers[j]->capturetime=(that->buffers[j]->leftCapture<that->buffers[j]->rightCapture ? that->buffers[j]->leftCapture : that->buffers[j]->rightCapture);
          that->conditionalBufferReleased.notify_all();
          that->numImages++;
        } else {
          // an error occurred during capturing. Ignore image
          EOUT ("Dropped frame: " << that->buffers[j]->frameLeft.Status << ", " << that->buffers[j]->frameRight.Status << std::endl);
          that->buffers[j]->capturetime.update();  // um zu verhindern, dass der selbe Puffer gleich wieder verwendet wird
          that->buffers[j]->bufferRead=true;
          that->buffers[j]->leftComplete = false;
          that->buffers[j]->rightComplete = false;
        }
        if (that->didStart) {
          that->putFrameOnBus();
        }
      }
      return;
    }
  }

  // frame not found in drivers. search frame in lostBuffers
  boost::unique_lock<boost::mutex> lockLostBuffers (GigEStereoCameraDriver::mutexLostBuffers);
  for (unsigned int i=0; i<GigEStereoCameraDriver::lostBuffers.size(); ++i) {
    if (&(GigEStereoCameraDriver::lostBuffers[i]->frameLeft)==frame) {
          GigEStereoCameraDriver::lostBuffers[i]->leftComplete = true;
    } else if (&(GigEStereoCameraDriver::lostBuffers[i]->frameRight)==frame) {
                 GigEStereoCameraDriver::lostBuffers[i]->rightComplete = true;
    }
    if (GigEStereoCameraDriver::lostBuffers[i]->leftComplete &&
        GigEStereoCameraDriver::lostBuffers[i]->rightComplete) {
      delete GigEStereoCameraDriver::lostBuffers[i];
      GigEStereoCameraDriver::lostBuffers.erase (GigEStereoCameraDriver::lostBuffers.begin()+i);
      return;
    }
  }
}


const ImageBuffer* DerWeg::GigEStereoCameraDriver::getImage (bool blocking) {
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
  buffers[iopt]->leftComplete = false;
  buffers[iopt]->rightComplete = false;
  return buffers[iopt];
}


void DerWeg::GigEStereoCameraDriver::releaseImage (const ImageBuffer* buf) {
  // search for buf in all drivers and all buffers
  boost::unique_lock<boost::mutex> lockDriversList (GigEStereoCameraDriver::mutexDriversList);
  bool buffer_found=false;
  for (unsigned int i=0; i<GigEStereoCameraDriver::driversList.size() && !buffer_found; ++i) {
    GigEStereoCameraDriver* that = driversList[i];
    boost::unique_lock<boost::mutex> lockBuffers (that->mutexBuffers);
    for (unsigned int j=0; j<that->buffers.size() && !buffer_found; ++j) {
      if (that->buffers[j]==buf) {

        that->buffers[j]->locks--;
        if (that->bufferOverflow && that->buffers[j]->locks==0) {
          // in case of buffer overflow immediately put buffer on bus
          that->buffers[j]->locks=-1;
          DerWeg::GigE::PvCaptureQueueFrame(that->leftHandle,&(that->buffers[j]->frameLeft),::DerWeg::stereoFrameDoneCallback);
          DerWeg::GigE::PvCaptureQueueFrame(that->rightHandle,&(that->buffers[j]->frameRight),::DerWeg::stereoFrameDoneCallback);
          that->bufferOverflow=false;
        }
        buffer_found=true;
      }
    }
  }
  if (!buffer_found) {
    boost::unique_lock<boost::mutex> lockLostBuffers (GigEStereoCameraDriver::mutexLostBuffers);
    // buf not found, search in lostBuffers and remove buffer if found
    for (unsigned int i=0; i<GigEStereoCameraDriver::lostBuffers.size(); ++i) {
      if (GigEStereoCameraDriver::lostBuffers[i]==buf) {
        delete GigEStereoCameraDriver::lostBuffers[i];
        GigEStereoCameraDriver::lostBuffers.erase (GigEStereoCameraDriver::lostBuffers.begin()+i);
        break;
      }
    }
  }
}


bool DerWeg::GigEStereoCameraDriver::startTransmission () throw() {
  boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
  bool okay = true;
  okay &= (DerWeg::GigE::PvCaptureStart(leftHandle)==ePvErrSuccess);
  okay &= (DerWeg::GigE::PvCaptureStart(rightHandle)==ePvErrSuccess);
  okay &= (DerWeg::GigE::PvCommandRun(leftHandle,"AcquisitionStart")==ePvErrSuccess);
  okay &= (DerWeg::GigE::PvCommandRun(rightHandle,"AcquisitionStart")==ePvErrSuccess);
  if (!okay) {
    DerWeg::GigE::PvCaptureEnd(leftHandle);
    DerWeg::GigE::PvCaptureEnd(rightHandle);
  }
  bufferOverflow = false;
  didStart=okay;
  numImages=0;
  captureStart.update();
  putFrameOnBus(BUFFERS_ON_BUS);
  return okay;
}


bool DerWeg::GigEStereoCameraDriver::stopTransmission () throw() {
  bool okay=true;
  {
    boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
    didStart=false;
    conditionalBufferReleased.notify_all();
    DerWeg::GigE::PvCommandRun(leftHandle,"AcquisitionStop");
    DerWeg::GigE::PvCommandRun(rightHandle,"AcquisitionStop");
    okay = (DerWeg::GigE::PvCaptureEnd(leftHandle)==ePvErrSuccess);
    okay &= (DerWeg::GigE::PvCaptureEnd(rightHandle)==ePvErrSuccess);
  }
  DerWeg::GigE::PvCaptureQueueClear(leftHandle);
  DerWeg::GigE::PvCaptureQueueClear(rightHandle);
  return okay;
}


double DerWeg::GigEStereoCameraDriver::frameRate () throw() {
  boost::unique_lock<boost::mutex> lockBuffers (mutexBuffers);
  return 1000*static_cast<double>(numImages)/static_cast<double>(captureStart.elapsed_msec());
}


unsigned int DerWeg::GigEStereoCameraDriver::maxImageWidth() throw (std::invalid_argument) {
  tPvUint32 minL, maxL, minR, maxR;
  if (DerWeg::GigE::PvAttrRangeUint32 (leftHandle, "Width", &minL, &maxL)!=ePvErrSuccess)
    throw std::invalid_argument ("error calling PvAttrRangeUint32 in DerWeg::GigEStereoCameraDriver::maxImageWidth");
  if (DerWeg::GigE::PvAttrRangeUint32 (rightHandle, "Width", &minR, &maxR)!=ePvErrSuccess)
    throw std::invalid_argument ("error calling PvAttrRangeUint32 in DerWeg::GigEStereoCameraDriver::maxImageWidth");
  return (maxL > maxR ? maxR : maxL);
}


unsigned int DerWeg::GigEStereoCameraDriver::maxImageHeight() throw (std::invalid_argument) {
  tPvUint32 minL, maxL, minR, maxR;
  if (DerWeg::GigE::PvAttrRangeUint32 (leftHandle, "Height", &minL, &maxL)!=ePvErrSuccess)
    throw std::invalid_argument ("error calling PvAttrRangeUint32 in DerWeg::GigEStereoCameraDriver::maxImageHeight");
  if (DerWeg::GigE::PvAttrRangeUint32 (rightHandle, "Height", &minR, &maxR)!=ePvErrSuccess)
    throw std::invalid_argument ("error calling PvAttrRangeUint32 in DerWeg::GigEStereoCameraDriver::maxImageHeight");
  return (maxL > maxR ? maxR : maxL);
}


bool DerWeg::GigEStereoCameraDriver::setWhiteBalance (unsigned int u, unsigned int v) throw() {
  return (
      (DerWeg::GigE::PvAttrEnumSet(leftHandle,"WhitebalMode","Manual")==ePvErrSuccess) &&
      (DerWeg::GigE::PvAttrUint32Set(leftHandle,"WhitebalRed",u)==ePvErrSuccess) &&
      (DerWeg::GigE::PvAttrUint32Set(leftHandle,"WhitebalBlue",v)==ePvErrSuccess) &&
      (DerWeg::GigE::PvAttrEnumSet(rightHandle,"WhitebalMode","Manual")==ePvErrSuccess) &&
      (DerWeg::GigE::PvAttrUint32Set(rightHandle,"WhitebalRed",u)==ePvErrSuccess) &&
      (DerWeg::GigE::PvAttrUint32Set(rightHandle,"WhitebalBlue",v)==ePvErrSuccess) );
}


bool DerWeg::GigEStereoCameraDriver::setAutoWhiteBalance () throw() {
  return (DerWeg::GigE::PvAttrEnumSet(leftHandle,"WhitebalMode","Auto")==ePvErrSuccess &&
        DerWeg::GigE::PvAttrEnumSet(rightHandle,"WhitebalMode","Auto")==ePvErrSuccess) ;
}


bool DerWeg::GigEStereoCameraDriver::setGain (unsigned int g) throw() {
  return (DerWeg::GigE::PvAttrUint32Set(leftHandle,"GainValue",g)==ePvErrSuccess) &&
        (DerWeg::GigE::PvAttrUint32Set(rightHandle,"GainValue",g)==ePvErrSuccess);
}


bool DerWeg::GigEStereoCameraDriver::setExposure (unsigned int e) throw() {
  return (
    (DerWeg::GigE::PvAttrEnumSet(leftHandle,"ExposureMode","Manual")==ePvErrSuccess) &&
    (DerWeg::GigE::PvAttrUint32Set(leftHandle,"ExposureValue",e)==ePvErrSuccess) &&
    (DerWeg::GigE::PvAttrEnumSet(rightHandle,"ExposureMode","Manual")==ePvErrSuccess) &&
    (DerWeg::GigE::PvAttrUint32Set(rightHandle,"ExposureValue",e)==ePvErrSuccess) );
}


bool DerWeg::GigEStereoCameraDriver::setAutoExposure (unsigned int e) throw() {
  return (
      (DerWeg::GigE::PvAttrEnumSet(leftHandle,"ExposureMode","Auto")==ePvErrSuccess) &&
      (DerWeg::GigE::PvAttrUint32Set(leftHandle,"ExposureAutoTarget",e)==ePvErrSuccess) &&
      (DerWeg::GigE::PvAttrEnumSet(rightHandle,"ExposureMode","Auto")==ePvErrSuccess) &&
      (DerWeg::GigE::PvAttrUint32Set(rightHandle,"ExposureAutoTarget",e)==ePvErrSuccess) );
}

