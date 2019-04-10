#pragma once

namespace ros 
{
enum Multiplication 
{
  M1 = 4,
  M2 = 2, 
  M4 = 1,
};

class Encorder 
{
  friend void callback();
public:
  Encorder(int pina, int pinb, Multiplication multiplication);
  Encorder();
  ~Encorder();
  void init();
  void countPulseCallback();
  int getPulse();
private:
  int pulse_;
  const int pina_, pinb_;
  const Multiplication multiplication_;
};
}

