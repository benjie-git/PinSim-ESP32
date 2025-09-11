#ifndef TB6612_h
#define TB6612_h

#include <Arduino.h>


class Solenoid
{
  public:
    Solenoid(int In1pin, int In2pin, int STBYpin);      

	  void fwd();
	  void rev();

    void coast();  // Stop motors, but allow them to coast to a halt.
    void brake();  // Stops motor by setting both input pins high
	
	  void standby();
	
  private:
  	int In1, In2, Standby;
};


#endif
