#ifndef DSUB_SLAVE_COMMUNICATOR_H
#define DSUB_SLAVE_COMMUNICATOR_H

#include "pin_event_detecter.hpp"
#include "func_event_detecter.hpp"
#include <Wire.h>
#include <Arduino.h>

/**
 *  @class DsubSlaveCommunicator
 *  @brief D-subを介した通信を管理するクラス、slave用
 */
class DsubSlaveCommunicator {
private:
  int _pin_goal_detect;                     //  ゴール検知ピン
  int _pin_hit_detect;                      //  コース接触検知ピン
  int _pin_goal_notify;                     //  ゴール通知ピン
  int _pin_hit_notify;                      //  コース接触通知ピン
  int _adress;                              //  スレーブアドレス
  bool (*_f_detect_goal)(void) = NULL;      //  ゴール検知関数
  bool (*_f_detect_hit)(void) = NULL;       //  コース接触関数
  EventDetecter *goalDetecter = NULL;       //  ゴール検知クラス
  EventDetecter *hitDetecter = NULL;        //  コース接触検知クラス

public:
  DsubSlaveCommunicator(int pin_goal_detect, int pin_hit_detect,
                  int pin_goal_notify, int pin_hit_notify, int adress, void (*f_on_receive)(int));
  DsubSlaveCommunicator(bool (*f_detect_goal)(void), bool (*f_detect_hit)(void),
                  int pin_goal_notify, int pin_hit_notify, int adress, void (*f_on_receive)(int));
  ~DsubSlaveCommunicator(void);
  bool setup_i2c(int adress, void (*f_on_receive)(int));
  bool handle_dsub_event(void);
};

#endif