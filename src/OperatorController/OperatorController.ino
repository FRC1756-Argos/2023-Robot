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

DebouncedButtonSet<2> inputs({
    DebouncedDigitalInput(OCInputs::button_left_middle_in,   // Input pin number
                          OCInputs::button_left_middle_gnd,  // Ground pin number
                          (bool)true),
    DebouncedDigitalInput(OCInputs::button_left_top_in,   // Input pin number
                          OCInputs::button_left_top_gnd,  // Ground pin number
                          (bool)true),

});

IndicatorLights<2> leds(
    {
        LED(OCOutputs::led_button_left_top_out, OCOutputs::led_button_left_top_gnd, OCOutputs::led_button_left_top_pwm),
        LED(OCOutputs::led_button_left_middle_out,
            OCOutputs::led_button_left_middle_gnd,
            OCOutputs::led_button_left_middle_pwm),
    },
    1);

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

  inputs.Update();
  leds.Update();

  const bool LeftTopVal = inputs.GetValue(OCInputs::button_left_top_idx);
  leds.SetPattern(OCOutputs::led_button_left_top_idx,
                  LeftTopVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool LeftMiddleVal = inputs.GetValue(OCInputs::button_left_middle_idx);
  leds.SetPattern(OCOutputs::led_button_left_middle_idx,
                  LeftMiddleVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool depressedPositionButton = LeftTopVal || LeftMiddleVal;

  if (LeftTopVal) {
    activePositionIdx = OCInputs::button_left_top_idx;
  } else if (LeftMiddleVal) {
    activePositionIdx = OCInputs::button_left_middle_idx;
  }

  Joystick.setButton(0, depressedPositionButton && 0 == activePositionIdx);
  Joystick.setButton(1, depressedPositionButton && 1 == activePositionIdx);
  //for (uint8_t index = GPButtons::gpButton_feeder_idx; index <= GPButtons::gpButton_cargoShip_idx; ++index) {
  //  Joystick.setButton(index, depressedPositionButton && index == activePositionIdx);
  //}

  switch (activePositionIdx) {
    case OCInputs::button_left_top_idx:
      leds.On(OCOutputs::led_button_left_top_idx);
      break;
    case OCInputs::button_left_middle_idx:
      leds.On(OCOutputs::led_button_left_middle_idx);
      break;
    default:  // No active position button
      break;
  }

  /*

  const bool feederVal = inputs.GetValue(OCInputs::button_feeder_idx);
  leds.SetPattern(OCOutputs::led_button_feeder_idx, feederVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);
  const bool depotVal = inputs.GetValue(OCInputs::button_depot_idx);
  leds.SetPattern(OCOutputs::led_button_depot_idx, depotVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);
  const bool nearRocketLowVal = inputs.GetValue(OCInputs::button_nearRocketLow_idx);
  leds.SetPattern(OCOutputs::led_button_nearRocketLow_idx,
                  nearRocketLowVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);
  const bool nearRocketMidVal = inputs.GetValue(OCInputs::button_nearRocketMid_idx);
  leds.SetPattern(OCOutputs::led_button_nearRocketMid_idx,
                  nearRocketMidVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);
  const bool nearRocketHighVal = inputs.GetValue(OCInputs::button_nearRocketHigh_idx);
  leds.SetPattern(OCOutputs::led_button_nearRocketHigh_idx,
                  nearRocketHighVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);
  const bool farRocketLowVal = inputs.GetValue(OCInputs::button_farRocketLow_idx);
  leds.SetPattern(OCOutputs::led_button_farRocketLow_idx,
                  farRocketLowVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);
  const bool farRocketMidVal = inputs.GetValue(OCInputs::button_farRocketMid_idx);
  leds.SetPattern(OCOutputs::led_button_farRocketMid_idx,
                  farRocketMidVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);
  const bool farRocketHighVal = inputs.GetValue(OCInputs::button_farRocketHigh_idx);
  leds.SetPattern(OCOutputs::led_button_farRocketHigh_idx,
                  farRocketHighVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);
  const bool cargoShipVal = inputs.GetValue(OCInputs::button_cargoShip_idx);
  leds.SetPattern(OCOutputs::led_button_cargoShip_idx,
                  cargoShipVal ? LED::LEDPattern::PULSE_FAST : LED::LEDPattern::SOLID);

  const bool depressedPositionButton = feederVal || depotVal || nearRocketLowVal || nearRocketMidVal ||
                                       nearRocketHighVal || farRocketLowVal || farRocketMidVal || farRocketHighVal ||
                                       cargoShipVal;

  // Prioritize by index and stop if a button has been pressed
  if (feederVal) {
    activePositionIdx = OCInputs::button_le;
  } else if (depotVal) {
    activePositionIdx = OCInputs::button_depot_idx;
  } else if (nearRocketLowVal) {
    activePositionIdx = OCInputs::button_nearRocketLow_idx;
  } else if (nearRocketMidVal) {
    activePositionIdx = OCInputs::button_nearRocketMid_idx;
  } else if (nearRocketHighVal) {
    activePositionIdx = OCInputs::button_nearRocketHigh_idx;
  } else if (farRocketLowVal) {
    activePositionIdx = OCInputs::button_farRocketLow_idx;
  } else if (farRocketMidVal) {
    activePositionIdx = OCInputs::button_farRocketMid_idx;
  } else if (farRocketHighVal) {
    activePositionIdx = OCInputs::button_farRocketHigh_idx;
  } else if (cargoShipVal) {
    activePositionIdx = OCInputs::button_cargoShip_idx;
  }

  for (uint8_t index = GPButtons::gpButton_feeder_idx; index <= GPButtons::gpButton_cargoShip_idx; ++index) {
    Joystick.setButton(index, depressedPositionButton && index == activePositionIdx);
  }

  // Limit scope to help catch copy-paste errors
  {
    const bool unassigned1Val = inputs.GetValue(OCInputs::button_unassigned1_idx);
    leds.Set(OCOutputs::led_button_unassigned1_idx, unassigned1Val);
    Joystick.setButton(GPButtons::gpButton_button1_idx, unassigned1Val);
  }
  {
    const bool unassigned2Val = inputs.GetValue(OCInputs::button_unassigned2_idx);
    leds.Set(OCOutputs::led_button_unassigned2_idx, unassigned2Val);
    Joystick.setButton(GPButtons::gpButton_button2_idx, unassigned2Val);
  }
  {
    const bool unassigned3Val = inputs.GetValue(OCInputs::button_unassigned3_idx);
    leds.Set(OCOutputs::led_button_unassigned3_idx, unassigned3Val);
    Joystick.setButton(GPButtons::gpButton_button3_idx, unassigned3Val);
  }
  {
    const bool unassigned4Val = inputs.GetValue(OCInputs::button_unassigned4_idx);
    leds.Set(OCOutputs::led_button_unassigned4_idx, unassigned4Val);
    Joystick.setButton(GPButtons::gpButton_button4_idx, unassigned4Val);
  }
  {
    const bool climbVal = inputs.GetValue(OCInputs::switch_climb_idx);
    leds.Set(OCOutputs::led_switch_climb_idx, climbVal);
    leds.SetPattern(OCOutputs::led_switch_climb_idx, LED::LEDPattern::PULSE_FAST);
    Joystick.setButton(GPButtons::gpButton_climb_idx, climbVal);
  }
  {
    const bool cargoHatchVal = inputs.GetValue(OCInputs::switch_cargoHatch_idx);
    leds.Set(OCOutputs::led_switch_cargoHatch_Cargo_idx, cargoHatchVal);
    leds.Set(OCOutputs::led_switch_cargoHatch_Hatch_idx, !cargoHatchVal);
    Joystick.setButton(GPButtons::gpButton_cargo_idx, cargoHatchVal);
    Joystick.setButton(GPButtons::gpButton_hatch_idx, !cargoHatchVal);
  }
  {
    const bool leftRightVal = inputs.GetValue(OCInputs::switch_leftRight_idx);
    leds.Set(OCOutputs::led_switch_leftRight_Left_idx, leftRightVal);
    leds.Set(OCOutputs::led_switch_leftRight_Right_idx, !leftRightVal);
    Joystick.setButton(GPButtons::gpButton_right_idx, leftRightVal);
    Joystick.setButton(GPButtons::gpButton_left_idx, !leftRightVal);
  }
  {
    const bool unassigned1SWVal = inputs.GetValue(OCInputs::switch_unassigned1_idx);
    leds.Set(OCOutputs::led_switch_unassigned1_ON_idx, unassigned1SWVal);
    leds.Set(OCOutputs::led_switch_unassigned1_OFF_idx, !unassigned1SWVal);
    Joystick.setButton(GPButtons::gpButton_switch1Right_idx, unassigned1SWVal);
    Joystick.setButton(GPButtons::gpButton_switch1Left_idx, !unassigned1SWVal);
  }
  {
    const bool unassigned2SWVal = inputs.GetValue(OCInputs::switch_unassigned2_idx);
    leds.Set(OCOutputs::led_switch_unassigned2_ON_idx, unassigned2SWVal);
    leds.Set(OCOutputs::led_switch_unassigned2_OFF_idx, !unassigned2SWVal);
    Joystick.setButton(GPButtons::gpButton_switch2Right_idx, unassigned2SWVal);
    Joystick.setButton(GPButtons::gpButton_switch2Left_idx, !unassigned2SWVal);
  }

  // Illuminate only one position button LED
  for (uint8_t ledIdx = OCOutputs::led_button_feeder_idx; ledIdx <= OCOutputs::led_button_cargoShip_idx; ++ledIdx) {
    leds.Off(ledIdx);
  }
  switch (activePositionIdx) {
    case OCInputs::button_feeder_idx:
      leds.On(OCOutputs::led_button_feeder_idx);
      break;
    case OCInputs::button_depot_idx:
      leds.On(OCOutputs::led_button_depot_idx);
      break;
    case OCInputs::button_nearRocketLow_idx:
      leds.On(OCOutputs::led_button_nearRocketLow_idx);
      break;
    case OCInputs::button_nearRocketMid_idx:
      leds.On(OCOutputs::led_button_nearRocketMid_idx);
      break;
    case OCInputs::button_nearRocketHigh_idx:
      leds.On(OCOutputs::led_button_nearRocketHigh_idx);
      break;
    case OCInputs::button_farRocketLow_idx:
      leds.On(OCOutputs::led_button_farRocketLow_idx);
      break;
    case OCInputs::button_farRocketMid_idx:
      leds.On(OCOutputs::led_button_farRocketMid_idx);
      break;
    case OCInputs::button_farRocketHigh_idx:
      leds.On(OCOutputs::led_button_farRocketHigh_idx);
      break;
    case OCInputs::button_cargoShip_idx:
      leds.On(OCOutputs::led_button_cargoShip_idx);
      break;
    default:  // No active position button
      break;
  }*/
}
