/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

////////////////////////////////////////////////////////////////////////////////
/// @file LED.h
///
/// @brief Manages a single LED
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

constexpr unsigned BLINK_FAST_PERIOD_MS = 500;
constexpr unsigned BLINK_SLOW_PERIOD_MS = 1000;
constexpr unsigned PULSE_FAST_PERIOD_MS = 250;
constexpr unsigned PULSE_SLOW_PERIOD_MS = 500;

constexpr unsigned char PWM_MIN_BRIGHTNESS = 64;
constexpr unsigned char PWM_MAX_BRIGHTNESS = 255;

constexpr unsigned PERIODDIVISOR = 16;

class LED {
 public:
  enum class LEDPattern {
    SOLID = 0,
    BLINK_FAST,
    BLINK_SLOW,
    PULSE_FAST,
    PULSE_SLOW,
  };

  explicit LED(uint8_t controlPin);
  LED(uint8_t controlPin, bool pwmEnable);
  LED(uint8_t controlPin, uint8_t groundPin, bool pwmEnable = false);

  void Initialize();
  void Update();

  void On();
  void Off();

  bool GetIlluminated() const;

  void SetPattern(LEDPattern newPattern);
  LEDPattern GetPattern() const;

  bool IsPWM() const;

 private:
  const uint8_t m_controlPin;
  const uint8_t m_groundPin;
  const bool m_pwmEnable;

  bool m_initialized;
  LEDPattern m_pattern;
  bool m_illuminate;

  static uint8_t GAMMA[256];
  static bool GAMMA_INITIALIZED;

  void SetPulse();
  void SetFlash();
  void SetSolid();

  uint8_t CalcPulseBrightness(uint8_t brightness, uint16_t period, bool smooth = false, uint8_t offBrightness = 0);
};
