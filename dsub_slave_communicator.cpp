#include "dsub_slave_communicator.hpp"
#include "debug.h"

/**
 * @fn コンストラクタ(検知ピン指定)
 * @param None
 * @return None
 */
DsubSlaveCommunicator::DsubSlaveCommunicator
  (int pin_goal_detect, int pin_hit_detect,
  int pin_goal_notify, int pin_hit_notify, int adress, void (*f_on_receive)(int))
  :_pin_goal_detect(pin_goal_detect)
  ,_pin_hit_detect(pin_hit_detect)
  ,_pin_goal_notify(pin_goal_notify)
  ,_pin_hit_notify(pin_hit_notify)
  ,_adress(adress)
{
  //  イベント検知インスタンス生成
  goalDetecter = new PinEventDetecter(_pin_goal_detect);
  hitDetecter = new PinEventDetecter(_pin_hit_detect);
  //  ピン動作モード設定
  //  検知系は入力
  pinMode(_pin_goal_detect, INPUT);
  pinMode(_pin_hit_detect, INPUT);
  //  通知系は出力
  pinMode(_pin_goal_notify, OUTPUT);
  pinMode(_pin_hit_notify, OUTPUT);
  this->setup_i2c(_adress, f_on_receive);
};

/**
 * @fn コンストラクタ(検知関数指定)
 * @param None
 * @return None
 */
DsubSlaveCommunicator::DsubSlaveCommunicator
  (bool (*f_detect_goal)(void), bool (*f_detect_hit)(void),
  int pin_goal_notify, int pin_hit_notify, int adress, void (*f_on_receive)(int))
  :_f_detect_goal(f_detect_goal)
  ,_f_detect_hit(f_detect_hit)
  ,_pin_goal_notify(pin_goal_notify)
  ,_pin_hit_notify(pin_hit_notify)
  ,_adress(adress)
{
  //  イベント検知インスタンス生成
  goalDetecter = new FuncEventDetecter(_f_detect_goal);
  hitDetecter = new FuncEventDetecter(_f_detect_hit);
  //  ピン動作モード設定
  //  通知系は出力
  pinMode(_pin_goal_notify, OUTPUT);
  pinMode(_pin_hit_notify, OUTPUT);
  this->setup_i2c(_adress, f_on_receive);
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
}

bool DsubSlaveCommunicator::setup_i2c(int adress, void (*f_on_receive)(int)){
  DebugPrint("i2c setup start");
  Wire.begin(adress);     //スレーブアドレスを取得してI2C開始
  Wire.onReceive(f_on_receive);
  DebugPrint("i2c setup end");
}