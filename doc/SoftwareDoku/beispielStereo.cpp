#include "toast2/stereo/kogmolabor.h"
// Tiefenschaetzer erzeugen:
DerWeg::StereoGPU tiefenschaetzer ("/home/common/calib.xml");
ImageBuffer ib;
cv::Mat depth, conf, disp, rect;
...
// die Tiefenschaetzung fuer Bildpaar ib starten
tiefenschaetzer.runStereoMatching (ib.image, ib.image_right);
// irgend etwas anderes tun (optional)
... 
// auf fertig berechnete Tiefenkarte warten und abholen
tiefenschaetzer.getStereoResults (depth, conf, disp, rect);