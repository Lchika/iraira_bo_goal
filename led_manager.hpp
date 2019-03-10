#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <ArduinoSTL.h>
#include <vector>
#include <initializer_list>
#include <Arduino.h>

/**
 * @class LedManager
 * @brief
 * 複数個のLEDを同時に操作するクラス\n
 * 基本的にはそれぞれのLEDに対して全く同じ操作を行うことを想定している
 */
class LedManager {
private:
  std::vector<int> pins;
  std::vector<bool> led_states;

public:
  LedManager(std::initializer_list<int> input_pins);
  void all_on(void);
  void all_off(void);
  void all_blink(int blink_time, int blink_count);
};

#endif