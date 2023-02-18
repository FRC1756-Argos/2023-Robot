/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

////////////////////////////////////////////////////////////////////////////////
/// @file PinMapping.h
///
/// @brief Pin mapping for the operator controller
///
/// @author David Turner (dkt01)
///
/// @copyright This file is part of 2019-Robot
///            (https://github.com/FRC1756-Argos/2019-Robot).
/// @copyright 2019-Robot is free software: you can redistribute it and/or modify
///            it under the terms of the GNU General Public License as published by
///            the Free Software Foundation, either version 3 of the License, or
///            (at your option) any later version.
/// @copyright 2019-Robot is distributed in the hope that it will be useful,
///            but WITHOUT ANY WARRANTY; without even the implied warranty of
///            MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
///            GNU General Public License for more details.
/// @copyright You should have received a copy of the GNU General Public License
///            along with 2019-Robot.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Arduino.h>

#include "MiscConstants.h"

namespace OCInputs {
  constexpr uint8_t button_left_top_in = A0;
  constexpr uint8_t button_left_top_gnd = A1;
  constexpr size_t button_left_top_idx = 0;

  constexpr uint8_t button_left_middle_in = 42;
  constexpr uint8_t button_left_middle_gnd = 43;
  constexpr size_t button_left_middle_idx = 1;

  constexpr uint8_t button_left_bottom_in = A2;
  constexpr uint8_t button_left_bottom_gnd = A3;
  constexpr size_t button_left_bottom_idx = 2;

  constexpr uint8_t button_middle_top_in = 38;
  constexpr uint8_t button_middle_top_gnd = 39;
  constexpr size_t button_middle_top_idx = 3;

  constexpr uint8_t button_middle_middle_in = 40;
  constexpr uint8_t button_middle_middle_gnd = 41;
  constexpr size_t button_middle_middle_idx = 4;

  constexpr uint8_t button_middle_bottom_in = 21;
  constexpr uint8_t button_middle_bottom_gnd = 22;
  constexpr size_t button_middle_bottom_idx = 5;

  constexpr uint8_t button_right_top_in = 16;
  constexpr uint8_t button_right_top_gnd = 13;
  constexpr size_t button_right_top_idx = 6;

  constexpr uint8_t button_right_middle_in = 50;
  constexpr uint8_t button_right_middle_gnd = 51;
  constexpr size_t button_right_middle_idx = 7;

  constexpr uint8_t button_right_bottom_in = 52;
  constexpr uint8_t button_right_bottom_gnd = 53;
  constexpr size_t button_right_bottom_idx = 8;

  constexpr uint8_t button_high_in = 19;
  constexpr uint8_t button_high_gnd = 20;
  constexpr size_t button_high_idx = 9;

  constexpr uint8_t button_middle_in = A8;
  constexpr uint8_t button_middle_gnd = A9;
  constexpr size_t button_middle_idx = 10;

  constexpr uint8_t button_low_in = 17;
  constexpr uint8_t button_low_gnd = 18;
  constexpr size_t button_low_idx = 11;

  constexpr uint8_t button_spare_in = 36;
  constexpr uint8_t button_spare_gnd = 37;
  constexpr size_t button_spare_idx = 12;

  constexpr uint8_t switch_led_in = 46;
  constexpr uint8_t switch_led_gnd = 47;
  constexpr size_t switch_led_idx = 13;

  constexpr uint8_t switch_gamep_in = 44;
  constexpr uint8_t switch_gamep_gnd = 45;
  constexpr size_t switch_gamep_idx = 14;

  constexpr uint8_t switch_bash_in = A6;
  constexpr uint8_t switch_bash_gnd = A7;
  constexpr size_t switch_bash_idx = 15;

  constexpr uint8_t switch_spare_in = A4;
  constexpr uint8_t switch_spare_gnd = A5;
  constexpr size_t switch_spare_idx = 16;

  constexpr uint8_t switch_boxboi_in = 49;
  constexpr uint8_t switch_boxboi_gnd = 48;
  constexpr size_t switch_boxboi_idx = 17;
}  // namespace OCInputs

namespace OCOutputs {

  constexpr uint8_t led_button_left_top_out = 26;
  constexpr uint8_t led_button_left_top_gnd = NOT_A_PIN;
  constexpr size_t led_button_left_top_idx = 0;
  constexpr bool led_button_left_top_pwm = false;

  constexpr uint8_t led_button_left_middle_out = 11;
  constexpr uint8_t led_button_left_middle_gnd = NOT_A_PIN;
  constexpr size_t led_button_left_middle_idx = 1;
  constexpr bool led_button_left_middle_pwm = true;

  constexpr uint8_t led_button_left_bottom_out = 27;
  constexpr uint8_t led_button_left_bottom_gnd = NOT_A_PIN;
  constexpr size_t led_button_left_bottom_idx = 2;
  constexpr bool led_button_left_bottom_pwm = false;

  constexpr uint8_t led_button_middle_top_out = 9;
  constexpr uint8_t led_button_middle_top_gnd = NOT_A_PIN;
  constexpr size_t led_button_middle_top_idx = 3;
  constexpr bool led_button_middle_top_pwm = true;

  constexpr uint8_t led_button_middle_middle_out = 10;
  constexpr uint8_t led_button_middle_middle_gnd = NOT_A_PIN;
  constexpr size_t led_button_middle_middle_idx = 4;
  constexpr bool led_button_middle_middle_pwm = true;

  constexpr uint8_t led_button_middle_bottom_out = 7;
  constexpr uint8_t led_button_middle_bottom_gnd = NOT_A_PIN;
  constexpr size_t led_button_middle_bottom_idx = 5;
  constexpr bool led_button_middle_bottom_pwm = true;

  constexpr uint8_t led_button_right_top_out = 4;
  constexpr uint8_t led_button_right_top_gnd = NOT_A_PIN;
  constexpr size_t led_button_right_top_idx = 6;
  constexpr bool led_button_right_top_pwm = true;

  constexpr uint8_t led_button_right_middle_out = 24;
  constexpr uint8_t led_button_right_middle_gnd = NOT_A_PIN;
  constexpr size_t led_button_right_middle_idx = 7;
  constexpr bool led_button_right_middle_pwm = false;

  constexpr uint8_t led_button_right_bottom_out = 25;
  constexpr uint8_t led_button_right_bottom_gnd = NOT_A_PIN;
  constexpr size_t led_button_right_bottom_idx = 8;
  constexpr bool led_button_right_bottom_pwm = false;

  constexpr uint8_t led_button_high_out = 6;
  constexpr uint8_t led_button_high_gnd = NOT_A_PIN;
  constexpr size_t led_button_high_idx = 9;
  constexpr bool led_button_high_pwm = true;

  constexpr uint8_t led_button_middle_out = 3;
  constexpr uint8_t led_button_middle_gnd = NOT_A_PIN;
  constexpr size_t led_button_middle_idx = 10;
  constexpr bool led_button_middle_pwm = true;

  constexpr uint8_t led_button_low_out = 5;
  constexpr uint8_t led_button_low_gnd = NOT_A_PIN;
  constexpr size_t led_button_low_idx = 11;
  constexpr bool led_button_low_pwm = true;

  constexpr uint8_t led_button_spare_out = 8;
  constexpr uint8_t led_button_spare_gnd = NOT_A_PIN;
  constexpr size_t led_button_spare_idx = 12;
  constexpr bool led_button_spare_pwm = true;

  constexpr uint8_t led_switch_led_out = 12;
  constexpr uint8_t led_switch_led_gnd = NOT_A_PIN;
  constexpr size_t led_switch_led_idx = 13;
  constexpr bool led_switch_led_pwm = true;

  constexpr uint8_t led_switch_gamep_on_out = 1;
  constexpr uint8_t led_switch_gamep_on_gnd = NOT_A_PIN;
  constexpr size_t led_switch_gamep_on_idx = 14;
  constexpr bool led_switch_gamep_on_pwm = false;

  constexpr uint8_t led_switch_gamep_off_out = 14;
  constexpr uint8_t led_switch_gamep_off_gnd = NOT_A_PIN;
  constexpr size_t led_switch_gamep_off_idx = 15;
  constexpr bool led_switch_gamep_off_pwm = false;

  constexpr uint8_t led_switch_bash_on_out = 30;
  constexpr uint8_t led_switch_bash_on_gnd = NOT_A_PIN;
  constexpr size_t led_switch_bash_on_idx = 16;
  constexpr bool led_switch_bash_on_pwm = false;

  constexpr uint8_t led_switch_bash_off_out = 31;
  constexpr uint8_t led_switch_bash_off_gnd = NOT_A_PIN;
  constexpr size_t led_switch_bash_off_idx = 17;
  constexpr bool led_switch_bash_off_pwm = false;

  constexpr uint8_t led_switch_spare_on_out = 28;
  constexpr uint8_t led_switch_spare_on_gnd = NOT_A_PIN;
  constexpr size_t led_switch_spare_on_idx = 18;
  constexpr bool led_switch_spare_on_pwm = false;

  constexpr uint8_t led_switch_spare_off_out = 29;
  constexpr uint8_t led_switch_spare_off_gnd = NOT_A_PIN;
  constexpr size_t led_switch_spare_off_idx = 19;
  constexpr bool led_switch_spare_off_pwm = false;

  constexpr uint8_t led_switch_boxboi_on_out = 3;
  constexpr uint8_t led_switch_boxboi_on_gnd = NOT_A_PIN;
  constexpr size_t led_switch_boxboi_on_idx = 20;
  constexpr bool led_switch_boxboi_on_pwm = true;

  constexpr uint8_t led_switch_boxboi_off_out = 14;
  constexpr uint8_t led_switch_boxboi_off_gnd = NOT_A_PIN;
  constexpr size_t led_switch_boxboi_off_idx = 21;
  constexpr bool led_switch_boxboi_off_pwm = false;

}  // namespace OCOutputs

namespace GPButtons {
  constexpr uint8_t numButtons = 18;

  constexpr uint8_t gpButton_left_top_idx = 0;
  constexpr uint8_t gpButton_left_middle_idx = 1;
  constexpr uint8_t gpButton_left_bottom_idx = 2;
  constexpr uint8_t gpButton_middle_top_idx = 3;
  constexpr uint8_t gpButton_middle_middle_idx = 4;
  constexpr uint8_t gpButton_middle_bottom_idx = 5;
  constexpr uint8_t gpButton_right_top_idx = 6;
  constexpr uint8_t gpButton_right_middle_idx = 7;
  constexpr uint8_t gpButton_right_bottom_idx = 8;
  constexpr uint8_t gpButton_high_idx = 9;
  constexpr uint8_t gpButton_middle_idx = 10;
  constexpr uint8_t gpButton_low_idx = 11;
  constexpr uint8_t gpButton_spare_idx = 12;
  constexpr uint8_t gpButton_led_idx = 13;
  constexpr uint8_t gpButton_gamep_idx = 14;
  constexpr uint8_t gpButton_bash_idx = 15;
  constexpr uint8_t gpButton_spareswitch_idx = 16;
  constexpr uint8_t gpButton_boxboi_idx = 17;
}  // namespace GPButtons
