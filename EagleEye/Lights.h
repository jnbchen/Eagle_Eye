#ifndef LIGHTS_H
#define LIGHTS_H


namespace DerWeg{

class Lights{

private:

  void SerialWrite(char Msg);

public:

  void left_indicator_on();
  void right_indicator_on();
  void indicator_off();
  void hazard_lights_on();
  void brake_light_on(int percentage);
  void brake_light_off();
};

}


#endif

