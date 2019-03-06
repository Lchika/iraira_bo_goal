#ifndef DSUB_SLAVE_COMMUNICATOR_H
#define DSUB_SLAVE_COMMUNICATOR_H

#include <Arduino.h>

/**
 *  @class DsubSlaveCommunicator
 *  @brief D-subを介した通信を管理するクラス、slave用
 */
class DsubSlaveCommunicator {
private:
  int _pin_goal_sensor;
  int _pin_hit_sensor;
  int _pin_goal;
  int _pin_hit;
  int _adress;

public:
  DsubSlaveCommunicator(int pin_goal_sensor, int pin_hit_sensor,
                  int pin_goal, int pin_hit, int adress);
  DsubSlaveCommunicator(F f);
  bool handle_dsub_event(void);
};

#endif