#include "solenoid.h"


Solenoid::Solenoid(int In1pin, int In2pin, int STBYpin)
{
   In1 = In1pin;
   In2 = In2pin;
   Standby = STBYpin;
}

void Solenoid::setup()
{
   pinMode(In1, OUTPUT);
   pinMode(In2, OUTPUT);
   if (Standby) {
      pinMode(Standby, OUTPUT);
   }
   this->standby();
}

void Solenoid::fwd()
{
   if (Standby) {
      digitalWrite(Standby, HIGH);
   }
   digitalWrite(In1, HIGH);
   digitalWrite(In2, LOW);
}

void Solenoid::rev()
{
   if (Standby) {
      digitalWrite(Standby, HIGH);
   }
   digitalWrite(In1, LOW);
   digitalWrite(In2, HIGH);
}

void Solenoid::brake()
{
   if (Standby) {
      digitalWrite(Standby, HIGH);
   }
   digitalWrite(In1, HIGH);
   digitalWrite(In2, HIGH);
}

void Solenoid::coast()
{
   if (Standby) {
      digitalWrite(Standby, HIGH);
   }
   digitalWrite(In1, LOW);
   digitalWrite(In2, LOW);
}

void Solenoid::standby()
{
   digitalWrite(In1, LOW);
   digitalWrite(In2, LOW);
   if (Standby) {
      digitalWrite(Standby, LOW);
   }
}
