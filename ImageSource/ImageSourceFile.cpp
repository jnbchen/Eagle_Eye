
#include <sstream>

#include "../Elementary/KogmoThread.h"
#include "../Elementary/ImageReaderWriter.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"


namespace DerWeg {

  /** implementation of ImageSource that reads images from a png-files */
  class ImageSourceFile : public KogmoThread {
    ImageReader ir;
    int cycle;      ///< Zykluszeit, in der Bilder Eingelesen werden sollen (in Millisekunden)
    void readImages();
    void execute ();
  public:
    ImageSourceFile(): cycle(100) {};
    ~ImageSourceFile() {};

    void init (const ConfigReader&);
  };

  void ImageSourceFile::init( const ConfigReader& cr) {
    std::string prefix;
    cr.get("ImagesFromFile::image_prefix", prefix);
    ir.setPrefix(prefix);
    std::string suffix=".png";
    cr.get("ImagesFromFile::image_suffix", suffix);
    ir.setSuffix(suffix);
    cr.get("ImagesFromFile::cycletime", cycle);
  }

  void ImageSourceFile::execute() {
    try{
      boost::timed_mutex tm;
      boost::unique_lock<boost::timed_mutex> lock (tm);
      boost::system_time timeout=boost::get_system_time() + boost::posix_time::milliseconds(cycle);
      ImageBuffer img1 = ir.getImage();
      BBOARD->setImage(img1);
      while (!img1.image.empty()) {
        tm.timed_lock (timeout);  // statt sleep
        boost::this_thread::interruption_point();
        timeout = timeout+boost::posix_time::milliseconds(cycle);
        // An Blackboard schicken
        img1 = ir.getImage();
        if (img1.image.empty()) {
          break;
        }
        BBOARD->setImage(img1);
      }
    }catch(boost::thread_interrupted&){ return; }
    EOUT("All available images read. Exit program." << std::endl);
    BBOARD->setExitProgram ();
  }

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::ImageSourceFile> imagesource_file ("ImagesFromFile");

} // namespace
