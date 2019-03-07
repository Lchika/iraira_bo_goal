#ifndef EVENT_DETECTER_H
#define EVENT_DETECTER_H

#include <Arduino.h>

/**
 *  @class EventDetecter
 *  @brief イベント検出インターフェース
 */
class EventDetecter {
private:

public:
  virtual ~EventDetecter(){};
  virtual bool is_event_detected() = 0;
};

#endif