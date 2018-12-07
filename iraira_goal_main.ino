/**
 * @file iraira_goal_main.ino
 * @brief イライラ棒ゴールコース制御プログラム
 * @date 2018.12.07
 * @details
 */

#include <StandardCplusplus.h>
#include "led_manager.hpp"
#include <FlexiTimer2.h>
#include <Servo.h>
#include <Arduino.h>

//  ピンアサイン
#define PIN_GOAL_SWITCH           2         //  ゴール前のマイクロスイッチ
#define PIN_SERVO                 3         //  O下部分のサーボ
#define PIN_GOAL_NOTIFI           4         //  ゴール通知
#define PIN_TOUCH_NOTIFI          5         //  コース接触通知
#define PIN_LED_TOP               7         //  ゴール上のLED
#define PIN_LED_BOTTOM            8         //  ゴール下のLED
#define PIN_COURSE_LEVEL          9         //  コース電圧レベル
#define PIN_PHOTO_INT             17        //  フォトインタラプタ入力

//  定数定義
#define BLINK_TIME_COURSE_TOUCH  50         //  LED点滅間隔[ms]
#define BLINK_COUNT_COURSE_TOUCH  3         //  LED点滅回数

#define INTERVAL_TIME_SERVO_CONTROL 50      //  サーボ制御処理間隔[ms]
#define CYCLE_TIME_SERVO_MOVE       4000    //  サーボ動作周期[ms]
#define HALF_CYCLE_TIME_SERVO_MOVE  (CYCLE_TIME_SERVO_MOVE / 2)
#define MIN_ANGLE_SERVO_MOVE        30      //  サーボ動作最小角度[度]
#define MAX_ANGLE_SERVO_MOVE        110     //  サーボ動作最大角度[度]
#define DEF_ANGLE_SERVO_MOVE        MIN_ANGLE_SERVO_MOVE

enum EVENT_E {
  EVENT_NONE = 0,             //  何もなし
  EVENT_COURSE_TOUCH,         //  コース接触
  EVENT_PHOTO_INT_THROUGH,    //  フォトインタラプタ通過
  EVENT_GOAL,                 //  ゴール
  EVENT_MAX,
};

//  関数定義
int get_event_state(void);
bool is_goal_switch_being_pushed(void);
bool is_course_being_touched(void);
bool is_passing_over_photo_int(void);
void start_servo_move(void);
void stop_servo_move(void);

//  変数定義
LedManager ledManager({PIN_LED_TOP, PIN_LED_BOTTOM});
Servo servo;
static bool is_servo_move = false;

/**
 * @fn セットアップ処理
 * @brief arduino起動時のセットアップ処理を行う
 * @param None
 * @return None
 * @sa
 * @detail
 */
void setup(){

  //  ピン設定初期化
  digitalWrite(PIN_GOAL_NOTIFI, LOW);
  digitalWrite(PIN_TOUCH_NOTIFI, LOW);

  pinMode(PIN_GOAL_SWITCH, INPUT);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_GOAL_NOTIFI, OUTPUT);
  pinMode(PIN_TOUCH_NOTIFI, OUTPUT);
  pinMode(PIN_COURSE_LEVEL, INPUT);
  pinMode(PIN_PHOTO_INT, INPUT);

  //  サーボの設定
  servo.attach(PIN_SERVO);
  servo.write(DEF_ANGLE_SERVO_MOVE);

  //  シリアル通信開始
  Serial.begin(115200);
  Serial.println("iraira_goal_main.ino start");

  return;
}

/**
 * @fn メインループ処理
 * @brief arduinoのメインループ処理を行う
 * @param None
 * @return None
 * @detail arduino起動後はsetup()実行後に本関数が繰り返し実行される
 * 最初に、現在発生しているイベントを確認する
 * イベントが発生していた場合、そのイベントに対応した処理を実行する
 */
void loop(){
  //  イベント確認
  int event = get_event_state();

  //  イベント対応処理実行
  exec_event_handler(event);

  return;
}

/**
 * @fn イベント状態取得処理
 * @brief
 * @param　None
 * @return イベント番号(詳細はEVENT_E参照)
 * @detail
 */
int get_event_state(void){
  //  ゴール前スイッチ通過中かどうか
  //  ゴール判定はほかの判定より厳密性を求められるため、割り込みの方がいいかも…
  if(is_goal_switch_being_pushed()){
    return EVENT_GOAL;
  }

  //  コース接触中かどうか
  if(is_course_being_touched()){
    return EVENT_COURSE_TOUCH;
  }

  //  フォトインタラプタ上通過中かどうか
  if(is_passing_over_photo_int()){
    return EVENT_PHOTO_INT_THROUGH;
  }

  //  どのイベントも起きていない
  return EVENT_NONE;
}

/**
 * @fn イベント対応作業実行処理
 * @brief
 * @param[in] event   イベント番号(詳細はEVENT_E参照)
 * @return None
 * @detail
 */
void exec_event_handler(int event){
  switch(event){
    //  ゴールした
    case EVENT_GOAL:
      //  ゴール地点LEDをすべて点灯
      ledManager.all_on();

      //  D-subゴール通知
      //  D-sub関係は共通モジュールにする予定なのでここには処理は書かない?

      //  状態初期化
      ledManager.all_off();
      stop_servo_move();

      break;

    //  コースに接触した
    case EVENT_COURSE_TOUCH:
      //  ゴール地点LEDを点滅
      ledManager.all_blink(BLINK_TIME_COURSE_TOUCH, BLINK_COUNT_COURSE_TOUCH);

      //  D-sub接触通知
      //  D-sub関係は共通モジュールにする予定なのでここには処理は書かない?

      break;

    //  フォトインタラプタを通過した
    case EVENT_PHOTO_INT_THROUGH:
      //  サーボ動作開始
      start_servo_move();
      break;
    
    //  何もなし
    case EVENT_NONE:
    default:
      break;
  }
}

/**
 * @fn ゴール前スイッチ状態確認処理
 * @brief
 * @param　None
 * @return true:スイッチが押されている, false:押されていない
 * @detail
 */
bool is_goal_switch_being_pushed(void){
  if(digitalRead(PIN_GOAL_SWITCH) == HIGH){
    return true;
  }
  return false;
}

/**
 * @fn コース接触確認処理
 * @brief
 * @param　None
 * @return true:接触している, false:接触していない
 * @detail
 */
bool is_course_being_touched(void){
  if(digitalRead(PIN_COURSE_LEVEL) == LOW){
    return true;
  }
  return false;
}

/**
 * @fn フォトインタラプタ通過確認処理
 * @brief
 * @param　None
 * @return true:通過中, false:通過中でない
 * @detail
 */
bool is_passing_over_photo_int(void){
  if(digitalRead(PIN_PHOTO_INT) == LOW){
    return true;
  }
  return false;
}

/**
 * @fn サーボ動作開始処理
 * @brief
 * @param　None
 * @return None
 * @detail
 */
void start_servo_move(void){
  if(!is_servo_move){
    //  サーボが動いていない状態の時のみ開始処理を行う
    //  タイマーセット
    FlexiTimer2::set(INTERVAL_TIME_SERVO_CONTROL, control_servo_move);
    FlexiTimer2::start();
    is_servo_move = true;
  }
  return;
}

/**
 * @fn サーボ動作停止処理
 * @brief
 * @param　None
 * @return None
 * @detail
 */
void stop_servo_move(void){
  if(is_servo_move){
    //  すでにサーボが動いている状態でのみ停止処理を行う
    //  タイマーストップ
    FlexiTimer2::stop();
    is_servo_move = false;
  }
  return;
}

/**
 * @fn サーボ動作制御処理
 * @brief
 * @param　None
 * @return None
 * @detail 一定時間ごとにこの関数を呼んでサーボを制御する
 */
void control_servo_move(void){
  float angle = DEF_ANGLE_SERVO_MOVE;
  
  int fixed_time = millis() % CYCLE_TIME_SERVO_MOVE;
  
  if(fixed_time < HALF_CYCLE_TIME_SERVO_MOVE){
    angle = (MAX_ANGLE_SERVO_MOVE - MIN_ANGLE_SERVO_MOVE) * ((float)fixed_time / HALF_CYCLE_TIME_SERVO_MOVE) + MIN_ANGLE_SERVO_MOVE;
  }else{
    angle = (MIN_ANGLE_SERVO_MOVE - MAX_ANGLE_SERVO_MOVE) * ((float)fixed_time / HALF_CYCLE_TIME_SERVO_MOVE - 1.0) + MAX_ANGLE_SERVO_MOVE;
  }

  servo.write(int(angle));
  return;
}