// msTimer
// 
// In loop, non-blocking, timer.
//
// Version 1.0


#ifndef MS_TIMER_H
#define MS_TIMER_H


#include <Arduino.h>

// Non-blocking millisecond timer.
class msTimer
{

private:
  unsigned long _oldMillis;
  unsigned long _delay;

public:
  // Default Constructor.
  msTimer()
  {
    _oldMillis = millis();
    _delay = 1000;
  }

  // Constructor.
  msTimer(unsigned long delay = 0)
  {
    _oldMillis = millis();
    _delay = delay;
  }

  // Returns true if delay has elapsed.
  // Reset delay.
  inline bool elapsed()
  {
    if ((_oldMillis + _delay) < millis())
    {
      _oldMillis = millis();
      return 1;
    }

    return 0;
  }

  inline void ForceTrigger()
  {
    _oldMillis = 0;
  }

  // Set delay and reset timer.
  inline void setDelayAndReset(unsigned long delay)
  {
    _delay = delay;
    _oldMillis = millis();
  }

  // Set delay and reset timer if delay is different.
  inline void setDelay(unsigned long delay)
  {
    if (_delay != delay)
    {
      _oldMillis = millis();
      _delay = delay;
    }
  }

  // Reset timer.
  inline void resetDelay()
  {
    _oldMillis = millis();
  }

  //deconstructor
  //~mstimer() {}
};

#endif