////////////////////////////////////////////////////////////////////////////////
/// @file LED.cpp
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

#include "LED.h"

bool LED::GAMMA_INITIALIZED = false;
uint8_t LED::GAMMA[256];

LED::LED( uint8_t controlPin) : LED( controlPin,
                                     (bool)false )
{}

LED::LED( uint8_t controlPin,
          bool    pwmEnable ) : LED( controlPin,
                                     NOT_A_PIN,
                                     pwmEnable )
{}

LED::LED( uint8_t controlPin,
          uint8_t groundPin,
          bool    pwmEnable ) : m_controlPin(controlPin),
                                m_groundPin(groundPin),
                                m_pwmEnable(pwmEnable),
                                m_initialized(false),
                                m_pattern(LEDPattern::SOLID),
                                m_illuminate(false)
{
  if(pwmEnable && !GAMMA_INITIALIZED)
  {
    GAMMA_INITIALIZED = true;
    // Populate Gamma correction lookup table
    // Gamma correction for x is:
    // ( x / MAXVAL )^2.5 * MAXVAL
    for(int i = 0; i < 256; i++)
    {
      float x = i;
      x /= 255;
      x = pow(x, 2.5);
      x *= 255;

      GAMMA[i] = x;
    }
  }
}

void LED::Initialize()
{
  if(!m_initialized)
  {
    if(NOT_A_PIN != m_controlPin)
    {
      pinMode(m_controlPin, OUTPUT);
      digitalWrite(m_controlPin, LOW);
    }
    if(NOT_A_PIN != m_groundPin)
    {
      pinMode(m_groundPin, OUTPUT);
      digitalWrite(m_groundPin, LOW);
    }
  }
  m_initialized = true;
}

void LED::Update()
{
  switch(m_pattern)
  {
    case LEDPattern::BLINK_FAST:
    case LEDPattern::BLINK_SLOW:
      SetFlash();
      break;
    case LEDPattern::PULSE_FAST:
    case LEDPattern::PULSE_SLOW:
      SetPulse();
      break;
    case LEDPattern::SOLID:
    default:
      SetSolid();
      break;
  }
}

void LED::On()
{
  m_illuminate = true;
  Update();
}

void LED::Off()
{
  m_illuminate = false;
  Update();
}

bool LED::GetIlluminated() const
{
  return m_illuminate;
}

void LED::SetPattern(LED::LEDPattern newPattern)
{
  m_pattern = newPattern;
}

LED::LEDPattern LED::GetPattern() const
{
  return m_pattern;
}

bool LED::IsPWM() const
{
  return m_pwmEnable;
}

void LED::SetPulse()
{
  if(!m_pwmEnable)
  {
    SetFlash();
    return;
  }

  if(NOT_A_PIN != m_controlPin)
  {
    uint8_t brightness = 0;
    if(m_illuminate)
    {
      uint16_t period = PULSE_SLOW_PERIOD_MS;
      switch(m_pattern)
      {
        case LEDPattern::PULSE_FAST:
          period = PULSE_FAST_PERIOD_MS;
          break;
        case LEDPattern::PULSE_SLOW:
          period = PULSE_SLOW_PERIOD_MS;
          break;
        case LEDPattern::BLINK_FAST:
        case LEDPattern::BLINK_SLOW:
        default:
          SetFlash();
          return;
          break;
      }
      brightness = CalcPulseBrightness( PWM_MAX_BRIGHTNESS,
                                        period,
                                        true,
                                        PWM_MIN_BRIGHTNESS );
    }
    analogWrite(m_controlPin, GAMMA[brightness]);
  }
}


void LED::SetFlash()
{
  if(NOT_A_PIN != m_controlPin)
  {
    const long now = millis();
    bool illuminate = m_illuminate;
    switch(m_pattern)
    {
      case LEDPattern::BLINK_FAST:
        illuminate = illuminate && (now / (BLINK_FAST_PERIOD_MS / 2)) % 2;
        break;
      case LEDPattern::PULSE_FAST:
        illuminate = illuminate && (now / (PULSE_FAST_PERIOD_MS / 2)) % 2;
        break;
      case LEDPattern::BLINK_SLOW:
        illuminate = illuminate && (now / (BLINK_SLOW_PERIOD_MS / 2)) % 2;
        break;
      case LEDPattern::PULSE_SLOW:
        illuminate = illuminate && (now / (PULSE_SLOW_PERIOD_MS / 2)) % 2;
        break;
      default:
        break;
    }
    digitalWrite(m_controlPin, illuminate ? HIGH : LOW);
  }
}

void LED::SetSolid()
{
  if(NOT_A_PIN != m_controlPin)
  {
    digitalWrite(m_controlPin, m_illuminate ? HIGH : LOW);
  }
}

uint8_t LED::CalcPulseBrightness( uint8_t brightness,
                                  uint16_t period,
                                  bool smooth,
                                  uint8_t offBrightness )
{
  bool invert = false;
  period /= PERIODDIVISOR;
  unsigned long condensedNow = millis() / PERIODDIVISOR;
  if(smooth)
  {
    invert = (condensedNow % (2 * period)) !=
             (condensedNow % period);
  }
  uint16_t curPhase = (condensedNow % period);
  if(invert)
  {
    curPhase = period - curPhase;
  }
  uint8_t brightnessRange = brightness - offBrightness;
  uint32_t outBrightness = curPhase * brightnessRange;
  outBrightness /= period;
  outBrightness += offBrightness;
  if(outBrightness > brightness)
  {
    outBrightness = brightness;
  }
  return static_cast<uint8_t>(outBrightness);
}

