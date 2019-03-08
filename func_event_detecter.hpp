#ifndef FUNC_EVENT_DETECTER_H
#define FUNC_EVENT_DETECTER_H

#include "event_detecter.hpp"
#include <Arduino.h>

/**
 *  @class FuncEventDetecter
 *  @brief 関数イベント検出クラス
 */
class FuncEventDetecter : public EventDetecter {
private:
  bool (*_event_func)(void) = NULL;

public:
  FuncEventDetecter(bool (*event_func)(void)):_event_func(event_func){};
  bool is_event_detected(void){
    if(_event_func != NULL){
      return _event_func();
    }else{
      return false;
    }
  };
};

#endif