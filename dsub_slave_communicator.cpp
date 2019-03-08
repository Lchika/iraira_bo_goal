#include "dsub_slave_communicator.hpp"
#include "debug.h"
#include <Wire.h>
#include <Arduino.h>

bool DsubSlaveCommunicator::_active = false;
int DsubSlaveCommunicator::_pin_goal_notify = -1;
int DsubSlaveCommunicator::_pin_hit_notify = -1;
char DsubSlaveCommunicator::dprint_buff[128];

/**
 * @fn コンストラクタ(検知ピン指定)
 * @param[in] pin_goal_detect ゴール検知ピン番号
 * @param[in] pin_hit_detect  コース接触検知ピン番号
 * @param[in] pin_goal_notify ゴール通知ピン番号
 * @param[in] pin_hit_notify  コース接触通知ピン番号
 * @param[in] adress          スレーブアドレス
 * @param[in] is_reverse_goal ゴール検知判定を反転させるか
 * @param[in] is_reverse_hit  コース接触判定を反転させるか
 * @return None
 */
DsubSlaveCommunicator::DsubSlaveCommunicator
  (int pin_goal_detect, int pin_hit_detect,
  int pin_goal_notify, int pin_hit_notify, unsigned char adress,
  bool is_reverse_goal, bool is_reverse_hit)
{
  DebugPrint("1");
  _pin_goal_notify = pin_goal_notify;
  _pin_hit_notify = pin_hit_notify;
  DebugPrint("2");
  //  イベント検知インスタンス生成
  goalDetecter = new PinEventDetecter(pin_goal_detect, is_reverse_goal);
  hitDetecter = new PinEventDetecter(pin_hit_detect, is_reverse_hit);
  //  ピン動作モード設定
  //  検知系は入力
  DebugPrint("3");
  pinMode(pin_goal_detect, INPUT);
  pinMode(pin_hit_detect, INPUT);
  //  通知系は出力
  pinMode(_pin_goal_notify, OUTPUT);
  pinMode(_pin_hit_notify, OUTPUT);
  DebugPrint("4");
  this->setup_i2c(adress);
};

/**
 * @fn コンストラクタ(検知関数指定)
 * @param
 * @return None
 */
DsubSlaveCommunicator::DsubSlaveCommunicator
  (bool (*f_detect_goal)(void), bool (*f_detect_hit)(void),
  int pin_goal_notify, int pin_hit_notify, unsigned char adress)
{
  _pin_goal_notify = pin_goal_notify;
  _pin_hit_notify = pin_hit_notify;
  //  イベント検知インスタンス生成
  goalDetecter = new FuncEventDetecter(f_detect_goal);
  hitDetecter = new FuncEventDetecter(f_detect_hit);
  //  ピン動作モード設定
  //  通知系は出力
  pinMode(_pin_goal_notify, OUTPUT);
  pinMode(_pin_hit_notify, OUTPUT);
  DsubSlaveCommunicator::setup_i2c(adress);
};

/**
 * @fn デストラクタ
 * @param None
 * @return None
 */
DsubSlaveCommunicator::~DsubSlaveCommunicator(void)
{
  if(goalDetecter != NULL){
    delete goalDetecter;
  }
  if(hitDetecter != NULL){
    delete hitDetecter;
  }
}

/**
 * @fn D-sub関係イベント処理関数
 * @param None
 * @return bool true:エラーなし、false:エラーあり
 * 
 * @detail
 * この関数はゴール・コース接触検知を行うため、定期的に呼ぶ必要がある
 * とりあえずはloop()内で実行しておけば大丈夫のはず
 * 丁寧なつくりにするならタイマを使って定期的に実行できるようにするとよい
 */
bool DsubSlaveCommunicator::handle_dsub_event(void)
{
  //  ゴール検知したとき
  if(goalDetecter->is_event_detected()){
    DebugPrint("goal detected");
    digitalWrite(_pin_goal_notify, HIGH);
  }

  //  コース接触検知したとき
  if(hitDetecter->is_event_detected()){
    DebugPrint("hit detected");
    digitalWrite(_pin_hit_notify, HIGH);
  }

  return true;
}

/**
 * @fn I2Cセットアップ関数
 * @param[in] adress  スレーブアドレス
 * @return bool true:エラーなし、false:エラーあり
 */
bool DsubSlaveCommunicator::setup_i2c(unsigned char adress){
  if(adress < 0){
    DebugPrint("invalid adress");
    return false;
  }

  DebugPrint("i2c setup start");
  sprintf(dprint_buff, "slave address = %d", adress);
  DebugPrint(dprint_buff);
  Wire.begin(adress);     //スレーブアドレスを取得してI2C開始
  Wire.onReceive(DsubSlaveCommunicator::handle_i2c_massage);
  DebugPrint("i2c setup end");

  return true;
}

/**
 * @fn I2Cメッセージ処理関数
 * @param[in] byte_num  受信メッセージバイト数
 * @return None
 * 参考:https://github.com/Lchika/IrairaBo_slavetemplate/blob/master/slave_template.ino
 */
void DsubSlaveCommunicator::handle_i2c_massage(int byte_num){
  DebugPrint("func start");
  if(_pin_goal_notify == -1 || _pin_hit_notify == -1){
    DebugPrint("<ERROR> invalid pin");
    return;
  }
  while(Wire.available()){
    DebugPrint("COMMON: got i2c massage");
    byte received_massage = Wire.read();
    switch(received_massage){
      case MASTER_BEGIN_TRANS:
        DebugPrint("COMMON: this module active");
        _active = true;
        pinMode(_pin_goal_notify, OUTPUT);    //  通過/ゴール判定ピンを出力に設定　
        pinMode(_pin_hit_notify, OUTPUT);     //  当たった判定ピンを出力に設定
        digitalWrite(_pin_goal_notify, LOW);
        digitalWrite(_pin_hit_notify, LOW);
        break;
      case MASTER_DETECT_GOAL:
        DebugPrint("COMMON: got MASTER_DETECT_GOAL");
        /* 通過/ゴール判定ピン、当たった判定ピンをLOWにしてから入力に切り替える */
        digitalWrite(_pin_goal_notify, LOW);
        digitalWrite(_pin_hit_notify, LOW);
        pinMode(_pin_goal_notify, INPUT);
        pinMode(_pin_hit_notify, INPUT);
        _active = false;
        break;
      case MASTER_DETECT_HIT:
        DebugPrint("COMMON: got MASTER_DETECT_HIT");
        digitalWrite(_pin_hit_notify, LOW);
        break;
      default:
        DebugPrint("COMMON: <ERROR>got default");
        break;
    }
  }
  DebugPrint("func end");
  return;
}

/**
 * @fn 活性状態確認関数
 * @param None
 * @return bool true:マスタから通信開始通知をもらっている, false:マスタから通信開始通知をもらっていない
 */
bool DsubSlaveCommunicator::is_active(void){
  return DsubSlaveCommunicator::_active;
}
