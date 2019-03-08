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
  bool _is_reverse;

public:
  PinEventDetecter(int event_pin, bool is_reverse = false)
  :_event_pin(event_pin)
  ,_is_reverse(is_reverse){};
  bool is_event_detected(void){
    int detect_level = HIGH;
    if(_is_reverse){
      detect_level = LOW;
    }
    if(digitalRead(_event_pin) == detect_level){
      return true;
    }else{
      return false;
    }
  };
};

#endif