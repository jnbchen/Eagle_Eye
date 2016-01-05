
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/ImageReaderWriter.h"


namespace DerWeg {

  /** an application that writes camera images to file */
  class SaveTopImage : public KogmoThread {
    ImageWriter writer;
    ImageBuffer ib;
  public:
    SaveTopImage () {;}
    void init (const ConfigReader& cfg) {
      std::string pf="image";
      cfg.get ("SaveTopImage::image_prefix", pf);
      writer.setPrefix(pf);
    }
    void execute () {
      try{
        while (true) {
          BBOARD->waitForTopImage();
          ib=BBOARD->getTopImage();
          if (!ib.image.empty()) {
            writer.setImage (ib);
          }

          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::SaveTopImage> application_save_camera_image ("SaveTopImages");

}
