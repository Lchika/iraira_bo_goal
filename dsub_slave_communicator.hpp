#ifndef DSUB_SLAVE_COMMUNICATOR_H
#define DSUB_SLAVE_COMMUNICATOR_H

#include "pin_event_detecter.hpp"
#include "func_event_detecter.hpp"
#include <ArduinoSTL.h>
#include <queue>

/**
 * @class DsubSlaveCommunicator
 * @brief D-subを介した通信を管理するクラス、slave用
 * 
 * このクラスを利用するにあたって把握しておくべきこと
 * - コンストラクタの引数について
 * - is_active()の使い方
 * - handle_dsub_event()の使い方
 * 
 * その他の部分はどのモジュールも共通のはずなので
 * 特に気にする必要はない(ようにこのクラスをつくります)
 */
class DsubSlaveCommunicator {
private:
  static const int I2C_BEGIN_TRANS = 0;     //  通信開始通知
  static const int I2C_DETECT_HIT  = 1;     //  コース接触通知確認通知
  static const int I2C_DETECT_GOAL = 2;     //  コース通過通知確認通知
  static const int I2C_EMPTY       = 99;
  static const int INTERVAL_DETECT_HIT_MS = 1000;
  static bool _active;                      //  マスタから通信開始通知をもらっているかどうか
  EventDetecter *goalDetecter = NULL;       //  ゴール検知クラス
  EventDetecter *hitDetecter = NULL;        //  コース接触検知クラス
  static char dprint_buff[];
  static std::queue<int> message_que;              //  送信用メッセージキュー

public:
  DsubSlaveCommunicator(int pin_goal_detect, int pin_hit_detect,
                  unsigned char adress, bool is_reverse_goal = false, bool is_reverse_hit = false);
  DsubSlaveCommunicator(bool (*f_detect_goal)(void), bool (*f_detect_hit)(void), unsigned char adress);
  ~DsubSlaveCommunicator(void);
  bool setup_i2c(unsigned char adress);
  bool handle_dsub_event(void);
  static void send_i2c_message(void);
  static void handle_i2c_message(int byte_num);
  static bool is_active(void);
};

#endif