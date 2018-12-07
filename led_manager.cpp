#include "led_manager.hpp"

LedManager::LedManager(std::initializer_list<int> input_pins)
   : pins(input_pins)
   , led_states({false, false, false})
{
  //  ピン設定の初期化
  for(const auto& pin : input_pins){
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
};

/**
 * @fn 全点灯処理
 * @brief
 * @param None
 * @return None
 * @detail 全部点ける
 */
void LedManager::all_on(void){
  for(const auto& pin : pins){
    digitalWrite(pin, HIGH);
  }
}

/**
 * @fn 全消灯処理
 * @brief
 * @param None
 * @return None
 * @detail 全部消す
 */
void LedManager::all_off(void){
  for(const auto& pin : pins){
    digitalWrite(pin, LOW);
  }
}

/**
 * @fn 全点滅処理
 * @brief
 * @param[in] blink_time    点滅時間[ms]
 * @param[in] blink_count   点滅回数
 * @return None
 * @detail blink_time分だけ点けて、消してをblink_count回繰り返す
 * delay()を使って時間処理を行うため、最低でもblink_time*2*blink_count[ms]はほかの処理を行えないことに注意
 */
void LedManager::all_blink(int blink_time, int blink_count){
  for(int i = 0; i < blink_count; i++){
    for(const auto& pin : pins){
      digitalWrite(pin, HIGH);
    }
    delay(blink_time);

    for(const auto& pin : pins){
      digitalWrite(pin, LOW);
    }
    delay(blink_time);
  }
}