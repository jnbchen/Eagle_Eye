#include "toast2/stereo/kogmolabor.h"
// Create depth approximator
DerWeg::StereoGPU tiefenschaetzer ("/home/common/calib.xml");
ImageBuffer ib;
cv::Mat depth, conf, disp, rect;
...
// Start depth approximation for ib image pair
tiefenschaetzer.runStereoMatching (ib.image, ib.image_right);
// Do something else (optional)
... 
// Wait for ready depth map and retrieve
tiefenschaetzer.getStereoResults (depth, conf, disp, rect);