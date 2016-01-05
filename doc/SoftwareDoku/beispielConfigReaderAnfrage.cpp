void init (const ConfigReader& cfg) {
  string colorspace;
  if (cfg.get("GigE::color_space", colorspace)) {
    LOUT ("color space is " << colorspace << endl)
  } else {
    LOUT ("parameter GigE::color_space not found" << endl)
  }
  int gain;
  if (cfg.get("GigE::gain", gain)) {
    LOUT ("gain is " << gain << endl)
  } else {
    LOUT ("parameter GigE::gain not found" << endl)
  }
  std::vector<int> imagesize;
  if (cfg.get("GigE::image_size", imagesize)) {
    if (imagesize.size()<4) {
      LOUT ("GigE::image_size has too few parameters" << endl)
    }
  } else {
    LOUT ("parameter GigE::image_size not found" << endl)
  }
}