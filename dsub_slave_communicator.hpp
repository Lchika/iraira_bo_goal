#ifndef DSUB_SLAVE_COMMUNICATOR_H
#define DSUB_SLAVE_COMMUNICATOR_H

#include "pin_event_detecter.hpp"
#include "func_event_detecter.hpp"

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
  static const int MASTER_BEGIN_TRANS = 0;  //  通信開始通知
  static const int MASTER_DETECT_HIT = 1;   //  コース接触通知確認通知
  static const int MASTER_DETECT_GOAL = 2;  //  コース通過通知確認通知
  static bool _active;                      //  マスタから通信開始通知をもらっているかどうか
  static int _pin_goal_notify;              //  ゴール通知ピン
  static int _pin_hit_notify;               //  コース接触通知ピン
  EventDetecter *goalDetecter = NULL;       //  ゴール検知クラス
  EventDetecter *hitDetecter = NULL;        //  コース接触検知クラス
  static char dprint_buff[];

public:
  DsubSlaveCommunicator(int pin_goal_detect, int pin_hit_detect,
                  int pin_goal_notify, int pin_hit_notify, unsigned char adress,
                  bool is_reverse_goal = false, bool is_reverse_hit = false);
  DsubSlaveCommunicator(bool (*f_detect_goal)(void), bool (*f_detect_hit)(void),
                  int pin_goal_notify, int pin_hit_notify, unsigned char adress);
  ~DsubSlaveCommunicator(void);
  bool setup_i2c(unsigned char adress);
  bool handle_dsub_event(void);
  static void handle_i2c_massage(int byte_num);
  static bool is_active(void);
};

#endif