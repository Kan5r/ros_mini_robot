#include "encorder.hpp"
#include <Arduino.h>

ros::Encorder::Encorder(int pina, int pinb, Multiplication multiplication) :
  pina_(pina),
  pinb_(pinb),
  multiplication_(multiplication)
{
  pulse_ = 0;
  init();
}

ros::Encorder::Encorder() :
  pina_(0),
  pinb_(0),
  multiplication_(ros::Multiplication::M4)
{
  
}

ros::Encorder::~Encorder() {}

void ros::Encorder::init() 
{
  pinMode(pina_, INPUT_PULLUP);
  pinMode(pinb_, INPUT_PULLUP);
}

void ros::Encorder::countPulseCallback() 
{
  static int pina_prev = 0; 
  static int pinb_prev = 0;

  int pina_now = digitalRead(pina_);
  int pinb_now = digitalRead(pinb_);
  
  if (pina_now != pina_prev) {
    switch (pina_now) {
    case HIGH:
      if (pinb_now == LOW) ++pulse_;
      else                 --pulse_;
      break;
    case LOW:
      if (pinb_now == HIGH) ++pulse_;
      else                  --pulse_;
      break;
    }
  } else if (pinb_now != pinb_prev) {
    switch (pinb_now) {
    case HIGH:
      if (pina_now == LOW) --pulse_;
      else                 ++pulse_;
      break;
    case LOW:
      if (pina_now == HIGH) --pulse_;
      else                  ++pulse_;
      break;
    }
  }
  
  pina_prev = pina_now;
  pinb_prev = pinb_now;
}

int ros::Encorder::getPulse()
{
  return pulse_;//round(pulse_ / multiplication_);
}
