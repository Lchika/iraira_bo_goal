#include "dsub_slave_communicator.hpp"
#include "debug.h"
#include <Wire.h>
#include <Arduino.h>

static const int I2C_BEGIN_TRANS    = 0;     //  通信開始通知
static const int I2C_DETECT_HIT     = 1;     //  コース接触通知確認通知
static const int I2C_DETECT_GOAL    = 2;     //  コース通過通知確認通知
static const int I2C_CHECK_CONNECT  = 3;     //  疎通確認
static const int I2C_EMPTY          = 99;

bool DsubSlaveCommunicator::_active = false;
char DsubSlaveCommunicator::dprint_buff[128];
std::queue<int> DsubSlaveCommunicator::message_que;

/**
 * @brief コンストラクタ(ピン指定)
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
  (int pin_goal_detect, int pin_hit_detect, unsigned char adress,
  bool is_reverse_goal, bool is_reverse_hit)
{
  //  イベント検知インスタンス生成
  goalDetecter = new PinEventDetecter(pin_goal_detect, is_reverse_goal);
  hitDetecter = new PinEventDetecter(pin_hit_detect, is_reverse_hit);
  //  ピン動作モード設定
  //  検知系は入力
  pinMode(pin_goal_detect, INPUT);
  pinMode(pin_hit_detect, INPUT);
  this->setup_i2c(adress);
  _active = false;
};

/**
 * @brief コンストラクタ(検知関数指定)
 * @param
 * @return None
 */
DsubSlaveCommunicator::DsubSlaveCommunicator
  (bool (*f_detect_goal)(void), bool (*f_detect_hit)(void), unsigned char adress)
{
  //  イベント検知インスタンス生成
  goalDetecter = new FuncEventDetecter(f_detect_goal);
  hitDetecter = new FuncEventDetecter(f_detect_hit);
  this->setup_i2c(adress);
  _active = false;
};

/**
 * @brief デストラクタ
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
 * @brief D-sub関係イベント処理関数
 * @param None
 * @return bool true:エラーなし、false:エラーあり
 * 
 * @details
 * この関数はゴール・コース接触検知を行うため、定期的に呼ぶ必要がある\n
 * とりあえずはloop()内で実行しておけば大丈夫のはず\n
 * 丁寧なつくりにするならタイマを使って定期的に実行できるようにするとよい
 */
bool DsubSlaveCommunicator::handle_dsub_event(void)
{
  static unsigned long last_hit_time = millis();

  //  ゴール検知したとき
  if(goalDetecter->is_event_detected()){
    DebugPrint("goal detected");
    message_que.push(I2C_DETECT_GOAL);
    _active = false;
  }

  //  コース接触検知したとき
  if(hitDetecter->is_event_detected()){
    unsigned long now_time = millis();
    if((now_time - last_hit_time) > INTERVAL_DETECT_HIT_MS){
      DebugPrint("hit detected");
      last_hit_time = millis();
      message_que.push(I2C_DETECT_HIT);
    }else{
      DebugPrint("hit detected(ignore)")
    }
  }

  return true;
}

/**
 * @brief I2Cセットアップ関数
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
  Wire.begin(adress);     //  スレーブアドレスを取得してI2C開始
  Wire.onRequest(DsubSlaveCommunicator::send_i2c_message);
  Wire.onReceive(DsubSlaveCommunicator::handle_i2c_message);
  DebugPrint("i2c setup end");

  return true;
}

/**
 * @brief I2Cメッセージ送信処理関数
 * @param[in] byte_num  受信メッセージバイト数
 * @return None
 * 
 * 参考:https://github.com/Lchika/IrairaBo_slavetemplate/blob/master/slave_template.ino
 */
void DsubSlaveCommunicator::send_i2c_message(void){
  //DebugPrint("func start");
  if(!message_que.empty()){
    sprintf(dprint_buff, "send i2c [%d]", message_que.front());
    DebugPrint(dprint_buff);
    Wire.write(message_que.front());
    message_que.pop();
  }else{
    //DebugPrint("send i2c [EMPTY]");
    Wire.write(I2C_EMPTY);
  }
  //DebugPrint("func end");
  return;
}

/**
 * @brief I2Cメッセージ処理関数
 * @param[in] byte_num  受信メッセージバイト数
 * @return None
 */
void DsubSlaveCommunicator::handle_i2c_message(int byte_num){
  DebugPrint("func start");
  while(Wire.available()){
    DebugPrint("got i2c massage");
    byte received_massage = Wire.read();
    switch(received_massage){
      case I2C_BEGIN_TRANS:
        DebugPrint("this module active");
        _active = true;
        break;
      
      //  疎通確認
      case I2C_CHECK_CONNECT:
        DebugPrint("check i2c connect");
        message_que.push(I2C_CHECK_CONNECT);
        break;
      
      default:
        DebugPrint("<ERROR> got default");
        break;
    }
  }
  DebugPrint("func end");
  return;
}

/**
 * @brief 活性状態確認関数
 * @param None
 * @return bool true:マスタから通信開始通知をもらっている, false:マスタから通信開始通知をもらっていない
 */
bool DsubSlaveCommunicator::is_active(void){
  return DsubSlaveCommunicator::_active;
}
