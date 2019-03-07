#ifndef PIN_EVENT_DETECTER_H
#define PIN_EVENT_DETECTER_H

#include "event_detecter.hpp"
#include <Arduino.h>

/**
 *  @class PinEventDetecter
 *  @brief ピンイベント検出クラス
 */
class PinEventDetecter : public EventDetecter {
private:
  int _event_pin;

public:
  PinEventDetecter(int event_pin):_event_pin(event_pin){};
  bool is_event_detected(void){
    if(digitalRead(_event_pin) == HIGH){
      return true;
    }else{
      return false;
    }
  };
};

#endif