/**
 * @file iraira_goal_main.ino
 * @brief イライラ棒ゴールコース制御プログラム
 * @date 2018.12.07
 * @details
 */

#include <ArduinoSTL.h>
#include "led_manager.hpp"
#include "dsub_slave_communicator.hpp"
#include "debug.h"
#include <FlexiTimer2.h>
#include <Servo.h>
#include <Arduino.h>

//  ピンアサイン
#define PIN_GOAL_SWITCH           2         //  ゴール前のマイクロスイッチ
#define PIN_SERVO                 3         //  O下部分のサーボ
#define PIN_GOAL_NOTIFY           4         //  ゴール通知
#define PIN_TOUCH_NOTIFY          5         //  コース接触通知
#define PIN_LED_TOP               7         //  ゴール上のLED
#define PIN_LED_BOTTOM            8         //  ゴール下のLED
#define PIN_COURSE_LEVEL          9         //  コース電圧レベル
#define PIN_10DIP_1               10        //  DIPロータリースイッチ入力1
#define PIN_10DIP_2               13        //  DIPロータリースイッチ入力2
#define PIN_10DIP_4               12        //  DIPロータリースイッチ入力4
#define PIN_10DIP_8               11        //  DIPロータリースイッチ入力8
#define PIN_PHOTO_INT             17        //  フォトインタラプタ入力

//  スレーブ共通部分
#define PIN_GOAL        PIN_GOAL_NOTIFY     //  ゴール判定用ピン
#define PIN_HIT         PIN_TOUCH_NOTIFY    //  当たった判定用ピン
#define PIN_GOAL_SENSOR PIN_GOAL_SWITCH     //  通過/ゴールしたことを検知するセンサのピン
#define PIN_HIT_SENSOR  PIN_COURSE_LEVEL    //  当たったことを検知するセンサのピン
#define MASTER_BEGIN_TRANS        0         //  通信を開始すること
#define MASTER_DETECT_HIT         1         //  HITを受信したこと
#define MASTER_DETECT_GOAL        2         //  通過/ゴールを受信したこと

//  定数定義
#define BLINK_TIME_COURSE_TOUCH   50        //  LED点滅間隔[ms]
#define BLINK_COUNT_COURSE_TOUCH  3         //  LED点滅回数

#define INTERVAL_TIME_SERVO_CONTROL 50      //  サーボ制御処理間隔[ms]
#define CYCLE_TIME_SERVO_MOVE       4000    //  サーボ動作周期[ms]
#define HALF_CYCLE_TIME_SERVO_MOVE  (CYCLE_TIME_SERVO_MOVE / 2)
#define MIN_ANGLE_SERVO_MOVE        30      //  サーボ動作最小角度[度]
#define MAX_ANGLE_SERVO_MOVE        110     //  サーボ動作最大角度[度]
#define DEF_ANGLE_SERVO_MOVE        MIN_ANGLE_SERVO_MOVE
#define SIZE_BUFF                   128     //  デバッグログ出力用一時バッファサイズ

enum EVENT_E {
  EVENT_NONE = 0,             //  何もなし
  EVENT_COURSE_TOUCH,         //  コース接触
  EVENT_PHOTO_INT_THROUGH,    //  フォトインタラプタ通過
  EVENT_GOAL,                 //  ゴール
  EVENT_MAX,
};

//  関数定義
unsigned char get_slave_address(void);
int get_event_state(void);
bool is_goal_switch_being_pushed(void);
bool is_course_being_touched(void);
bool is_passing_over_photo_int(void);
void start_servo_move(void);
void stop_servo_move(void);
static void i2c_massage_handle(int byte_num);

//  変数定義
LedManager ledManager({PIN_LED_TOP, PIN_LED_BOTTOM});   //  LED管理用インスタンス
Servo servo;                                            //  サーボ制御用インスタンス
static bool is_servo_move = false;                      //  サーボを動作させるか[true:動作させる, false:動作させない]
DsubSlaveCommunicator *dsubSlaveCommunicator;           //  D-sub通信管理用インスタンス
char dprint_buff[SIZE_BUFF];                            //  デバッグログ用一時バッファ

/**
 * @brief セットアップ処理
 * @param None
 * @return None
 * @sa
 * @details arduino起動時のセットアップ処理を行う
 */
void setup(){
  /* ここから各スレーブ共通コード */
  pinMode(PIN_GOAL, INPUT);     //通過判定用ピンを入力として設定
  pinMode(PIN_HIT, INPUT);      //当たった判定用ピンを入力として設定
  /* ここまで各スレーブ共通コード */

  //  シリアル通信開始
  BeginDebugPrint();
  DebugPrint("iraira_goal_main.ino start");

  //  ピン設定初期化
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_PHOTO_INT, INPUT);

  //  サーボの設定
  servo.attach(PIN_SERVO);
  servo.write(DEF_ANGLE_SERVO_MOVE);

  //  D-sub通信用インスタンス生成
  //  ログを出したいのでここで生成する
  dsubSlaveCommunicator = new DsubSlaveCommunicator
                          (PIN_GOAL_SWITCH, PIN_COURSE_LEVEL,
                          get_slave_address(), false, true);

  return;
}

/**
 * @brief メインループ処理
 * @param None
 * @return None
 * @details
 * arduino起動後はsetup()実行後に本関数が繰り返し実行される\n
 * 最初に、現在発生しているイベントを確認する\n
 * イベントが発生していた場合、そのイベントに対応した処理を実行する
 */
void loop(){
  //  マスタから通信開始通知が来ている場合
  if(DsubSlaveCommunicator::is_active()){
    //  D-sub関係イベント処理
    dsubSlaveCommunicator->handle_dsub_event();

    //  イベント確認
    int event = get_event_state();

    //  イベント対応処理実行
    exec_event_handler(event);
  }
  //  マスタから通信開始通知が来ていない場合は何もしない
  return;
}

/**
 * @brief I2Cスレーブアドレス取得処理
 * @param　None
 * @return None
 */
unsigned char get_slave_address(void){
  unsigned char adress = digitalRead(PIN_10DIP_1) | (digitalRead(PIN_10DIP_2) << 1) |
                       (digitalRead(PIN_10DIP_4) << 2) | (digitalRead(PIN_10DIP_8) << 3);
  sprintf(dprint_buff, "slave address = %d", adress);
  DebugPrint(dprint_buff);
  return adress;
}

/**
 * @brief イベント状態取得処理
 * @param　None
 * @return イベント番号(詳細はEVENT_E参照)
 */
int get_event_state(void){
  //  ゴール前スイッチ通過中かどうか
  //  ゴール判定はほかの判定より厳密性を求められるため、割り込みの方がいいかも…
  if(is_goal_switch_being_pushed()){
    DebugPrint("get event GOAL");
    return EVENT_GOAL;
  }

  //  コース接触中かどうか
  if(is_course_being_touched()){
    DebugPrint("get event COURSE TOUCH");
    return EVENT_COURSE_TOUCH;
  }

  //  フォトインタラプタ上通過中かどうか
  if(is_passing_over_photo_int()){
    DebugPrint("get event PHOTO INT THROUGH");
    return EVENT_PHOTO_INT_THROUGH;
  }

  //  どのイベントも起きていない
  return EVENT_NONE;
}

/**
 * @brief イベント対応作業実行処理
 * @param[in] event   イベント番号(詳細はEVENT_E参照)
 * @return None
 */
void exec_event_handler(int event){
  switch(event){
    //  ゴールした
    case EVENT_GOAL:
      DebugPrint("handle event GOAL");
      //  ゴール地点LEDをすべて点灯
      ledManager.all_on();
      DebugPrint("led all on");

      //  D-subゴール通知
      //  D-sub関係は共通モジュールにする予定なのでここには処理は書かない?

      //  状態初期化
      DebugPrint("initialize start");
      ledManager.all_off();
      stop_servo_move();
      DebugPrint("initialize end");

      break;

    //  コースに接触した
    case EVENT_COURSE_TOUCH:
      DebugPrint("handle event COURSE TOUCH");
      //  ゴール地点LEDを点滅
      sprintf(dprint_buff, "led_all_blink[%d, %d]", BLINK_TIME_COURSE_TOUCH, BLINK_COUNT_COURSE_TOUCH);
      //txt = "led all blink[" + String(BLINK_TIME_COURSE_TOUCH) + ", " + String(BLINK_COUNT_COURSE_TOUCH) + "]";
      DebugPrint(dprint_buff);
      ledManager.all_blink(BLINK_TIME_COURSE_TOUCH, BLINK_COUNT_COURSE_TOUCH);

      //  D-sub接触通知
      //  D-sub関係は共通モジュールにする予定なのでここには処理は書かない?

      break;

    //  フォトインタラプタを通過した
    case EVENT_PHOTO_INT_THROUGH:
      DebugPrint("handle event PHOTO INT THROUGH");
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
 * @brief ゴール前スイッチ状態確認処理
 * @param　None
 * @return true:スイッチが押されている, false:押されていない
 */
bool is_goal_switch_being_pushed(void){
  if(digitalRead(PIN_GOAL_SWITCH) == HIGH){
    return true;
  }
  return false;
}

/**
 * @brief コース接触確認処理
 * @param　None
 * @return true:接触している, false:接触していない
 */
bool is_course_being_touched(void){
  if(digitalRead(PIN_COURSE_LEVEL) == LOW){
    return true;
  }
  return false;
}

/**
 * @brief フォトインタラプタ通過確認処理
 * @param　None
 * @return true:通過中, false:通過中でない
 */
bool is_passing_over_photo_int(void){
  if(digitalRead(PIN_PHOTO_INT) == LOW){
    return true;
  }
  return false;
}

/**
 * @brief サーボ動作開始処理
 * @param　None
 * @return None
 */
void start_servo_move(void){
  if(!is_servo_move){
    //  サーボが動いていない状態の時のみ開始処理を行う
    //  タイマーセット
    FlexiTimer2::set(INTERVAL_TIME_SERVO_CONTROL, control_servo_move);
    FlexiTimer2::start();
    is_servo_move = true;
    DebugPrint("start servo move");
  }
  return;
}

/**
 * @brief サーボ動作停止処理
 * @param　None
 * @return None
 */
void stop_servo_move(void){
  if(is_servo_move){
    //  すでにサーボが動いている状態でのみ停止処理を行う
    //  タイマーストップ
    FlexiTimer2::stop();
    is_servo_move = false;
    DebugPrint("stop servo move");
  }
  return;
}

/**
 * @brief サーボ動作制御処理
 * @param　None
 * @return None
 * @details 一定時間ごとにこの関数を呼んでサーボを制御する
 */
void control_servo_move(void){
  float angle = DEF_ANGLE_SERVO_MOVE;
  
  int fixed_time = millis() % CYCLE_TIME_SERVO_MOVE;
  
  if(fixed_time < HALF_CYCLE_TIME_SERVO_MOVE){
    angle = (MAX_ANGLE_SERVO_MOVE - MIN_ANGLE_SERVO_MOVE) *
            ((float)fixed_time / HALF_CYCLE_TIME_SERVO_MOVE) + MIN_ANGLE_SERVO_MOVE;
  }else{
    angle = (MIN_ANGLE_SERVO_MOVE - MAX_ANGLE_SERVO_MOVE) *
            ((float)fixed_time / HALF_CYCLE_TIME_SERVO_MOVE - 1.0) + MAX_ANGLE_SERVO_MOVE;
  }

  servo.write(int(angle));
  return;
}
