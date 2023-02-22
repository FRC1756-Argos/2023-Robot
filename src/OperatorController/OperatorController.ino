////////////////////////////////////////////////////////////////////////////////
/// @file OperatorController.ino
///
/// @brief Operator controller firmware for Argos team 1756's 2019 robot
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

#include <Joystick.h>
#include "DebouncedButtonSet.h"
#include "IndicatorLights.h"
#include "PinMapping.h"

constexpr uint16_t debounceSamples = 5;

DebouncedButtonSet<18> inputs({
    DebouncedDigitalInput(OCInputs::button_left_top_in,   // Input pin number
                          OCInputs::button_left_top_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_left_middle_in,   // Input pin number
                          OCInputs::button_left_middle_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_left_bottom_in,   // Input pin number
                          OCInputs::button_left_bottom_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_middle_top_in,   // Input pin number
                          OCInputs::button_middle_top_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_middle_middle_in,   // Input pin number
                          OCInputs::button_middle_middle_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_middle_bottom_in,   // Input pin number
                          OCInputs::button_middle_bottom_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_right_top_in,   // Input pin number
                          OCInputs::button_right_top_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_right_middle_in,   // Input pin number
                          OCInputs::button_right_middle_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_right_bottom_in,   // Input pin number
                          OCInputs::button_right_bottom_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_high_in,   // Input pin number
                          OCInputs::button_high_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_middle_in,   // Input pin number
                          OCInputs::button_middle_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_low_in,   // Input pin number
                          OCInputs::button_low_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_stow_in,   // Input pin number
                          OCInputs::button_stow_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::switch_led_in,   // Input pin number
                          OCInputs::switch_led_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::switch_gamep_in,   // Input pin number
                          OCInputs::switch_gamep_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::switch_bash_in,   // Input pin number
                          OCInputs::switch_bash_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::switch_spare_in,   // Input pin number
                          OCInputs::switch_spare_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::switch_boxboi_in,   // Input pin number
                          OCInputs::switch_boxboi_gnd,  // Ground pin number
                          (bool)true)
});

IndicatorLights<22> leds(
    {
        LED(OCOutputs::led_button_left_top_out,
            OCOutputs::led_button_left_top_gnd,
            OCOutputs::led_button_left_top_pwm),
        LED(OCOutputs::led_button_left_middle_out,
            OCOutputs::led_button_left_middle_gnd,
            OCOutputs::led_button_left_middle_pwm),
        LED(OCOutputs::led_button_left_bottom_out,
            OCOutputs::led_button_left_bottom_gnd,
            OCOutputs::led_button_left_bottom_pwm),
        LED(OCOutputs::led_button_middle_top_out,
            OCOutputs::led_button_middle_top_gnd,
            OCOutputs::led_button_middle_top_pwm),
        LED(OCOutputs::led_button_middle_middle_out,
            OCOutputs::led_button_middle_middle_gnd,
            OCOutputs::led_button_middle_middle_pwm),
        LED(OCOutputs::led_button_middle_bottom_out,
            OCOutputs::led_button_middle_bottom_gnd,
            OCOutputs::led_button_middle_bottom_pwm),
        LED(OCOutputs::led_button_right_top_out,
            OCOutputs::led_button_left_top_gnd,
            OCOutputs::led_button_right_top_pwm),
        LED(OCOutputs::led_button_right_middle_out,
            OCOutputs::led_button_right_middle_gnd,
            OCOutputs::led_button_right_middle_pwm),
        LED(OCOutputs::led_button_right_bottom_out,
            OCOutputs::led_button_right_bottom_gnd,
            OCOutputs::led_button_right_bottom_pwm),
        LED(OCOutputs::led_button_high_out,
            OCOutputs::led_button_high_gnd,
            OCOutputs::led_button_high_pwm),
        LED(OCOutputs::led_button_middle_out,
            OCOutputs::led_button_middle_gnd,
            OCOutputs::led_button_middle_pwm),
        LED(OCOutputs::led_button_low_out,
            OCOutputs::led_button_low_gnd,
            OCOutputs::led_button_low_pwm),
        LED(OCOutputs::led_button_stow_out,
            OCOutputs::led_button_stow_gnd,
            OCOutputs::led_button_stow_pwm),
        LED(OCOutputs::led_switch_led_out,
            OCOutputs::led_switch_led_gnd,
            OCOutputs::led_switch_led_pwm),
        LED(OCOutputs::led_switch_gamep_on_out,
            OCOutputs::led_switch_gamep_on_gnd,
            OCOutputs::led_switch_gamep_on_pwm),
        LED(OCOutputs::led_switch_gamep_off_out,
            OCOutputs::led_switch_gamep_off_gnd,
            OCOutputs::led_switch_gamep_off_pwm),
        LED(OCOutputs::led_switch_bash_on_out,
            OCOutputs::led_switch_bash_on_gnd,
            OCOutputs::led_switch_bash_on_pwm),
        LED(OCOutputs::led_switch_bash_off_out,
            OCOutputs::led_switch_bash_off_gnd,
            OCOutputs::led_switch_bash_off_pwm),
        LED(OCOutputs::led_switch_spare_on_out,
            OCOutputs::led_switch_spare_on_gnd,
            OCOutputs::led_switch_spare_on_pwm),
        LED(OCOutputs::led_switch_spare_off_out,
            OCOutputs::led_switch_spare_off_gnd,
            OCOutputs::led_button_stow_pwm),
        LED(OCOutputs::led_switch_boxboi_on_out,
            OCOutputs::led_switch_boxboi_on_gnd,
            OCOutputs::led_switch_boxboi_on_pwm),
        LED(OCOutputs::led_switch_boxboi_off_out,
            OCOutputs::led_switch_boxboi_off_gnd,
            OCOutputs::led_switch_boxboi_off_pwm),


    },
    10);

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
                   JOYSTICK_TYPE_GAMEPAD,
                   GPButtons::numButtons,
                   0,  // Button Count, Hat Switch Count
                   false,
                   false,
                   false,  // No X, Y, or Z axis
                   false,
                   false,
                   false,  // No Rx, Ry, or Rz
                   false,
                   false,  // No rudder or throttle
                   false,
                   false,
                   false);  // No accelerator, brake, or steering

void setup() {
  // Initialize Button Pins
  inputs.Initialize();
  inputs.SetDebounceCount(debounceSamples);
  leds.Initialize();
  for (size_t i = 0; i < leds.GetNumLights(); ++i) {
    leds.SetPattern(i, LED::LEDPattern::SOLID);
  }

  // Test illuminate all LEDs
  for (uint8_t activeIdx = 0; activeIdx < leds.GetNumLights(); ++activeIdx) {
    for (uint8_t setIdx = 0; setIdx < leds.GetNumLights(); ++setIdx) {
      leds.Set(setIdx, setIdx == activeIdx);
    }
    leds.Update();
    delay(100);
  }


  // Initialize Joystick Library
  Joystick.begin();
}

void loop() {
  static uint8_t activePositionIdx = NOT_A_PIN;
  static uint8_t activePositionHighIdx = NOT_A_PIN;

  inputs.Update();
  leds.Update();

  const bool LeftTopVal = inputs.GetValue(OCInputs::button_left_top_idx);
  leds.SetPattern(OCOutputs::led_button_left_top_idx,
                  LeftTopVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool LeftMiddleVal = inputs.GetValue(OCInputs::button_left_middle_idx);
  leds.SetPattern(OCOutputs::led_button_left_middle_idx,
                  LeftMiddleVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool LeftBottomVal = inputs.GetValue(OCInputs::button_left_bottom_idx);
  leds.SetPattern(OCOutputs::led_button_left_bottom_idx,
                  LeftBottomVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool MiddleTopVal = inputs.GetValue(OCInputs::button_middle_top_idx);
  leds.SetPattern(OCOutputs::led_button_middle_top_idx,
                  MiddleTopVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool MiddleMiddleVal = inputs.GetValue(OCInputs::button_middle_middle_idx);
  leds.SetPattern(OCOutputs::led_button_middle_middle_idx,
                  MiddleMiddleVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool MiddleBottomVal = inputs.GetValue(OCInputs::button_middle_bottom_idx);
  leds.SetPattern(OCOutputs::led_button_middle_bottom_idx,
                  MiddleBottomVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool RightTopVal = inputs.GetValue(OCInputs::button_right_top_idx);
  leds.SetPattern(OCOutputs::led_button_right_top_idx,
                  RightTopVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool RightMiddleVal = inputs.GetValue(OCInputs::button_right_middle_idx);
  leds.SetPattern(OCOutputs::led_button_right_middle_idx,
                  RightMiddleVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool RightBottomVal = inputs.GetValue(OCInputs::button_right_bottom_idx);
  leds.SetPattern(OCOutputs::led_button_right_bottom_idx,
                  RightBottomVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool HighVal = inputs.GetValue(OCInputs::button_high_idx);
  leds.SetPattern(OCOutputs::led_button_high_idx,
                  HighVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool MiddleVal = inputs.GetValue(OCInputs::button_middle_idx);
  leds.SetPattern(OCOutputs::led_button_middle_idx,
                  MiddleVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool LowVal = inputs.GetValue(OCInputs::button_low_idx);
  leds.SetPattern(OCOutputs::led_button_low_idx,
                  LowVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool StowVal = inputs.GetValue(OCInputs::button_stow_idx);
  leds.SetPattern(OCOutputs::led_button_stow_idx,
                  StowVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool LedVal = inputs.GetValue(OCInputs::switch_led_idx);
  leds.Set(OCOutputs::led_switch_led_idx, LedVal);
  leds.SetPattern(OCOutputs::led_switch_led_idx, LED::LEDPattern::PULSE_FAST);
  Joystick.setButton(GPButtons::gpButton_led_idx, LedVal);

  const bool GamepVal = inputs.GetValue(OCInputs::switch_gamep_idx);
  leds.Set(OCOutputs::led_switch_gamep_on_idx, GamepVal);
  leds.Set(OCOutputs::led_switch_gamep_off_idx, !GamepVal);
  // leds.SetPattern(OCOutputs::led_switch_gamep_on_idx, LED::LEDPattern::PULSE_FAST);
  Joystick.setButton(GPButtons::gpButton_gamep_idx, GamepVal);

  const bool BashVal = inputs.GetValue(OCInputs::switch_bash_idx);
  leds.Set(OCOutputs::led_switch_bash_on_idx, BashVal);
  leds.Set(OCOutputs::led_switch_bash_off_idx, !BashVal);
  // leds.SetPattern(OCOutputs::led_switch_gamep_on_idx, LED::LEDPattern::PULSE_FAST);
  Joystick.setButton(GPButtons::gpButton_bash_idx, BashVal);

  const bool SpareSwitchVal = inputs.GetValue(OCInputs::switch_spare_idx);
  leds.Set(OCOutputs::led_switch_spare_on_idx, SpareSwitchVal);
  leds.Set(OCOutputs::led_switch_spare_off_idx, !SpareSwitchVal);
  // leds.SetPattern(OCOutputs::led_switch_gamep_on_idx, LED::LEDPattern::PULSE_FAST);
  Joystick.setButton(GPButtons::gpButton_spareswitch_idx, SpareSwitchVal);

  const bool depressedPositionButton = LeftTopVal || LeftMiddleVal || LeftBottomVal || MiddleTopVal || MiddleMiddleVal || MiddleBottomVal || RightTopVal || RightMiddleVal || RightBottomVal || StowVal;

  if (LeftTopVal) {
    activePositionIdx = OCInputs::button_left_top_idx;
  } else if (LeftMiddleVal) {
    activePositionIdx = OCInputs::button_left_middle_idx;
  } else if (LeftBottomVal) {
    activePositionIdx = OCInputs::button_left_bottom_idx;
  } else if (MiddleTopVal) {
    activePositionIdx = OCInputs::button_middle_top_idx;
  } else if (MiddleMiddleVal) {
    activePositionIdx = OCInputs::button_middle_middle_idx;
  } else if (MiddleBottomVal) {
    activePositionIdx = OCInputs::button_middle_bottom_idx;
  } else if (RightTopVal) {
    activePositionIdx = OCInputs::button_right_top_idx;
  } else if (RightMiddleVal) {
    activePositionIdx = OCInputs::button_right_middle_idx;
  } else if (RightBottomVal) {
    activePositionIdx = OCInputs::button_right_bottom_idx;
  } else if (StowVal) {
    activePositionIdx = OCInputs::button_stow_idx;
  }

  for (uint8_t index = GPButtons::gpButton_left_top_idx; index <= GPButtons::gpButton_stow_idx; ++index) {
   Joystick.setButton(index, depressedPositionButton && index == activePositionIdx);
  }

  switch (activePositionIdx) {
    case OCInputs::button_left_top_idx:
      leds.On(OCOutputs::led_button_left_top_idx);
      break;
    case OCInputs::button_left_middle_idx:
      leds.On(OCOutputs::led_button_left_middle_idx);
      break;
    case OCInputs::button_left_bottom_idx:
      leds.On(OCOutputs::led_button_left_bottom_idx);
      break;
    case OCInputs::button_middle_top_idx:
      leds.On(OCOutputs::led_button_middle_top_idx);
      break;
    case OCInputs::button_middle_middle_idx:
      leds.On(OCOutputs::led_button_middle_middle_idx);
      break;
    case OCInputs::button_middle_bottom_idx:
      leds.On(OCOutputs::led_button_middle_bottom_idx);
      break;
    case OCInputs::button_right_top_idx:
      leds.On(OCOutputs::led_button_right_top_idx);
      break;
    case OCInputs::button_right_middle_idx:
      leds.On(OCOutputs::led_button_right_middle_idx);
      break;
    case OCInputs::button_right_bottom_idx:
      leds.On(OCOutputs::led_button_right_bottom_idx);
      break;
    case OCInputs::button_stow_idx:
      leds.On(OCOutputs::led_button_stow_idx);
      break;
    default:  // No active position button
      break;
  }

for (uint8_t index = GPButtons::gpButton_left_top_idx; index <= GPButtons::gpButton_stow_idx; ++index) {
 if(index != activePositionIdx){
  leds.Off(index);
 }
}
 const bool depressedPositionButtonHeight = HighVal || MiddleVal || LowVal;

   if (HighVal) {
    activePositionHighIdx = OCInputs::button_high_idx;
    } else if (MiddleVal) {
    activePositionHighIdx = OCInputs::button_middle_idx;
    } else if (LowVal) {
    activePositionHighIdx = OCInputs::button_low_idx;
    }

  for (uint8_t index = GPButtons::gpButton_high_idx; index <= GPButtons::gpButton_low_idx; ++index) {
   Joystick.setButton(index, depressedPositionButtonHeight && index == activePositionHighIdx);
  }

  switch (activePositionHighIdx) {
    case OCInputs::button_high_idx:
      leds.On(OCOutputs::led_button_high_idx);
      break;
    case OCInputs::button_middle_idx:
      leds.On(OCOutputs::led_button_middle_idx);
      break;
    case OCInputs::button_low_idx:
      leds.On(OCOutputs::led_button_low_idx);
      break;
      default: // no active position button
      break;
  }

for (uint8_t index = GPButtons::gpButton_high_idx; index <= GPButtons::gpButton_low_idx; ++index) {
 if(index != activePositionHighIdx){
  leds.Off(index);
  }
 }
}
