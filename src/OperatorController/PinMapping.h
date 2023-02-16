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

#ifndef PIN_MAPPING_H
#define PIN_MAPPING_H

#include <Arduino.h>
#include "MiscConstants.h"

namespace OCInputs {
  constexpr uint8_t button_left_top_in = A0;
  constexpr uint8_t button_left_top_gnd = A1;
  constexpr size_t button_left_top_idx = 0;

  constexpr uint8_t button_left_middle_in = 42;
  constexpr uint8_t button_left_middle_gnd = 43;
  constexpr size_t button_left_middle_idx = 1;

  /*
  constexpr uint8_t button_feeder_in = A8;
  constexpr uint8_t button_feeder_gnd = A9;
  constexpr uint8_t button_depot_in = 16;
  constexpr uint8_t button_depot_gnd = 13;
  constexpr uint8_t button_nearRocketLow_in = 17;
  constexpr uint8_t button_nearRocketLow_gnd = 18;
  constexpr size_t button_nearRocketLow_idx = 2;
  constexpr uint8_t button_nearRocketMid_in = 19;
  constexpr uint8_t button_nearRocketMid_gnd = 20;
  constexpr size_t button_nearRocketMid_idx = 3;
  constexpr uint8_t button_nearRocketHigh_in = 21;
  constexpr uint8_t button_nearRocketHigh_gnd = 22;
  constexpr size_t button_nearRocketHigh_idx = 4;
  constexpr uint8_t button_farRocketLow_in = 36;
  constexpr uint8_t button_farRocketLow_gnd = 37;
  constexpr size_t button_farRocketLow_idx = 5;
  constexpr uint8_t button_farRocketMid_in = 38;
  constexpr uint8_t button_farRocketMid_gnd = 39;
  constexpr size_t button_farRocketMid_idx = 6;
  constexpr uint8_t button_farRocketHigh_in = 40;
  constexpr uint8_t button_farRocketHigh_gnd = 41;
  constexpr size_t button_farRocketHigh_idx = 7;

  constexpr size_t button_cargoShip_idx = 8;
  constexpr uint8_t switch_cargoHatch_in = 44;
  constexpr uint8_t switch_cargoHatch_gnd = 45;
  constexpr size_t switch_cargoHatch_idx = 9;
  constexpr uint8_t switch_climb_in = 46;
  constexpr uint8_t switch_climb_gnd = 47;
  constexpr size_t switch_climb_idx = 10;
  constexpr uint8_t switch_leftRight_in = 48;
  constexpr uint8_t switch_leftRight_gnd = 49;
  constexpr size_t switch_leftRight_idx = 11;
  constexpr uint8_t button_unassigned1_in = 50;
  constexpr uint8_t button_unassigned1_gnd = 51;
  constexpr size_t button_unassigned1_idx = 12;
  constexpr uint8_t button_unassigned2_in = 52;
  constexpr uint8_t button_unassigned2_gnd = 53;
  constexpr size_t button_unassigned2_idx = 13;

  constexpr size_t button_unassigned3_idx = 14;
  constexpr uint8_t button_unassigned4_in = A2;
  constexpr uint8_t button_unassigned4_gnd = A3;
  constexpr size_t button_unassigned4_idx = 15;
  constexpr uint8_t switch_unassigned1_in = A4;
  constexpr uint8_t switch_unassigned1_gnd = A5;
  constexpr size_t switch_unassigned1_idx = 16;
  constexpr uint8_t switch_unassigned2_in = A6;
  constexpr uint8_t switch_unassigned2_gnd = A7;
  constexpr size_t switch_unassigned2_idx = 17;
  constexpr uint8_t extra_unassigned1_in = A8;
  constexpr uint8_t extra_unassigned1_gnd = A9;
  constexpr size_t extra_unassigned1_idx = 18;
  constexpr uint8_t extra_unassigned2_in = A10;
  constexpr uint8_t extra_unassigned2_gnd = A11;
  constexpr size_t extra_unassigned2_idx = 19;
  constexpr uint8_t extra_unassigned3_in = DAC0;
  constexpr uint8_t extra_unassigned3_gnd = DAC1;
  constexpr size_t extra_unassigned3_idx = 20;
  constexpr uint8_t extra_unassigned4_in = CANRX;
  constexpr uint8_t extra_unassigned4_gnd = CANTX;
  constexpr size_t extra_unassigned4_idx = 21;
  constexpr uint8_t extra_unassigned5_in = SDA1;
  constexpr uint8_t extra_unassigned5_gnd = SCL1;
  constexpr size_t extra_unassigned5_idx = 22;*/
}  // namespace OCInputs

namespace OCOutputs {

  constexpr uint8_t led_button_left_top_out = 26;
  constexpr uint8_t led_button_left_top_gnd = NOT_A_PIN;
  constexpr size_t led_button_left_top_idx = 0;
  constexpr bool led_button_left_top_pwm = true;

  constexpr uint8_t led_button_left_middle_out = 11;
  constexpr uint8_t led_button_left_middle_gnd = NOT_A_PIN;
  constexpr size_t led_button_left_middle_idx = 1;
  constexpr bool led_button_left_middle_pwm = true;

  /*
  constexpr uint8_t led_button_feeder_out = 3;
  constexpr uint8_t led_button_feeder_gnd = NOT_A_PIN;
  constexpr uint8_t led_button_depot_out = 4;
  constexpr uint8_t led_button_depot_gnd = NOT_A_PIN;
  constexpr bool led_button_depot_pwm = true;
  constexpr size_t led_button_depot_idx = 1;
  constexpr uint8_t led_button_nearRocketLow_out = 5;
  constexpr uint8_t led_button_nearRocketLow_gnd = NOT_A_PIN;
  constexpr bool led_button_nearRocketLow_pwm = true;
  constexpr size_t led_button_nearRocketLow_idx = 2;
  constexpr uint8_t led_button_nearRocketMid_out = 6;
  constexpr uint8_t led_button_nearRocketMid_gnd = NOT_A_PIN;
  constexpr bool led_button_nearRocketMid_pwm = true;
  constexpr size_t led_button_nearRocketMid_idx = 3;
  constexpr uint8_t led_button_nearRocketHigh_out = 7;
  constexpr uint8_t led_button_nearRocketHigh_gnd = NOT_A_PIN;
  constexpr bool led_button_nearRocketHigh_pwm = true;
  constexpr size_t led_button_nearRocketHigh_idx = 4;
  constexpr uint8_t led_button_farRocketLow_out = 8;
  constexpr uint8_t led_button_farRocketLow_gnd = NOT_A_PIN;
  constexpr bool led_button_farRocketLow_pwm = true;
  constexpr size_t led_button_farRocketLow_idx = 5;
  constexpr uint8_t led_button_farRocketMid_out = 9;
  constexpr uint8_t led_button_farRocketMid_gnd = NOT_A_PIN;
  constexpr bool led_button_farRocketMid_pwm = true;
  constexpr size_t led_button_farRocketMid_idx = 6;
  constexpr uint8_t led_button_farRocketHigh_out = 10;
  constexpr uint8_t led_button_farRocketHigh_gnd = NOT_A_PIN;
  constexpr bool led_button_farRocketHigh_pwm = true;
  constexpr size_t led_button_farRocketHigh_idx = 7;

  constexpr uint8_t led_switch_climb_out = 12;
  constexpr uint8_t led_switch_climb_gnd = NOT_A_PIN;
  constexpr bool led_switch_climb_pwm = true;
  constexpr size_t led_switch_climb_idx = 9;
  constexpr uint8_t led_switch_cargoHatch_Cargo_out = 1;
  constexpr uint8_t led_switch_cargoHatch_Cargo_gnd = NOT_A_PIN;
  constexpr bool led_switch_cargoHatch_Cargo_pwm = false;
  constexpr size_t led_switch_cargoHatch_Cargo_idx = 10;
  constexpr uint8_t led_switch_cargoHatch_Hatch_out = 14;
  constexpr uint8_t led_switch_cargoHatch_Hatch_gnd = NOT_A_PIN;
  constexpr bool led_switch_cargoHatch_Hatch_pwm = false;
  constexpr size_t led_switch_cargoHatch_Hatch_idx = 11;
  constexpr uint8_t led_switch_leftRight_Left_out = 23;
  constexpr uint8_t led_switch_leftRight_Left_gnd = NOT_A_PIN;
  constexpr bool led_switch_leftRight_Left_pwm = false;
  constexpr size_t led_switch_leftRight_Left_idx = 12;
  constexpr uint8_t led_switch_leftRight_Right_out = 15;
  constexpr uint8_t led_switch_leftRight_Right_gnd = NOT_A_PIN;
  constexpr bool led_switch_leftRight_Right_pwm = false;
  constexpr size_t led_switch_leftRight_Right_idx = 13;
  constexpr uint8_t led_button_unassigned1_out = 24;
  constexpr uint8_t led_button_unassigned1_gnd = NOT_A_PIN;
  constexpr bool led_button_unassigned1_pwm = false;
  constexpr size_t led_button_unassigned1_idx = 14;
  constexpr uint8_t led_button_unassigned2_out = 25;
  constexpr uint8_t led_button_unassigned2_gnd = NOT_A_PIN;
  constexpr bool led_button_unassigned2_pwm = false;
  constexpr size_t led_button_unassigned2_idx = 15;

  constexpr bool led_button_unassigned3_pwm = false;
  constexpr size_t led_button_unassigned3_idx = 16;
  constexpr uint8_t led_button_unassigned4_out = 27;
  constexpr uint8_t led_button_unassigned4_gnd = NOT_A_PIN;
  constexpr bool led_button_unassigned4_pwm = false;
  constexpr size_t led_button_unassigned4_idx = 17;
  constexpr uint8_t led_switch_unassigned1_OFF_out = 29;
  constexpr uint8_t led_switch_unassigned1_OFF_gnd = NOT_A_PIN;
  constexpr bool led_switch_unassigned1_OFF_pwm = false;
  constexpr size_t led_switch_unassigned1_OFF_idx = 18;
  constexpr uint8_t led_switch_unassigned1_ON_out = 28;
  constexpr uint8_t led_switch_unassigned1_ON_gnd = NOT_A_PIN;
  constexpr bool led_switch_unassigned1_ON_pwm = false;
  constexpr size_t led_switch_unassigned1_ON_idx = 19;
  constexpr uint8_t led_switch_unassigned2_OFF_out = 31;
  constexpr uint8_t led_switch_unassigned2_OFF_gnd = NOT_A_PIN;
  constexpr bool led_switch_unassigned2_OFF_pwm = false;
  constexpr size_t led_switch_unassigned2_OFF_idx = 20;
  constexpr uint8_t led_switch_unassigned2_ON_out = 30;
  constexpr uint8_t led_switch_unassigned2_ON_gnd = NOT_A_PIN;
  constexpr bool led_switch_unassigned2_ON_pwm = false;
  constexpr size_t led_switch_unassigned2_ON_idx = 21;
  constexpr uint8_t led_extra_unassigned1_out = 32;
  constexpr uint8_t led_extra_unassigned1_gnd = NOT_A_PIN;
  constexpr bool led_extra_unassigned1_pwm = false;
  constexpr size_t led_extra_unassigned1_idx = 22;
  constexpr uint8_t led_extra_unassigned2_out = 33;
  constexpr uint8_t led_extra_unassigned2_gnd = NOT_A_PIN;
  constexpr bool led_extra_unassigned2_pwm = false;
  constexpr size_t led_extra_unassigned2_idx = 23;
  constexpr uint8_t led_extra_unassigned3_out = 34;
  constexpr uint8_t led_extra_unassigned3_gnd = NOT_A_PIN;
  constexpr bool led_extra_unassigned3_pwm = false;
  constexpr size_t led_extra_unassigned3_idx = 24;
  constexpr uint8_t led_extra_unassigned4_out = 35;
  constexpr uint8_t led_extra_unassigned4_gnd = NOT_A_PIN;
  constexpr bool led_extra_unassigned4_pwm = false;
  constexpr size_t led_extra_unassigned4_idx = 25;*/
}  // namespace OCOutputs

namespace GPButtons {
  constexpr uint8_t numButtons = 22;

  /*
  constexpr uint8_t gpButton_feeder_idx = 0;
  constexpr uint8_t gpButton_depot_idx = 1;
  constexpr uint8_t gpButton_nearRocketLow_idx = 2;
  constexpr uint8_t gpButton_nearRocketMid_idx = 3;
  constexpr uint8_t gpButton_nearRocketHigh_idx = 4;
  constexpr uint8_t gpButton_farRocketLow_idx = 5;
  constexpr uint8_t gpButton_farRocketMid_idx = 6;
  constexpr uint8_t gpButton_farRocketHigh_idx = 7;
  constexpr uint8_t gpButton_cargoShip_idx = 8;
  constexpr uint8_t gpButton_left_idx = 9;
  constexpr uint8_t gpButton_right_idx = 10;
  constexpr uint8_t gpButton_hatch_idx = 11;
  constexpr uint8_t gpButton_cargo_idx = 12;
  constexpr uint8_t gpButton_climb_idx = 13;
  constexpr uint8_t gpButton_switch1Left_idx = 14;
  constexpr uint8_t gpButton_switch1Right_idx = 15;
  constexpr uint8_t gpButton_switch2Left_idx = 16;
  constexpr uint8_t gpButton_switch2Right_idx = 17;
  constexpr uint8_t gpButton_button1_idx = 18;
  constexpr uint8_t gpButton_button2_idx = 19;
  constexpr uint8_t gpButton_button3_idx = 20;
  constexpr uint8_t gpButton_button4_idx = 21;*/
}  // namespace GPButtons

#endif
