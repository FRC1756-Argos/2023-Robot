////////////////////////////////////////////////////////////////////////////////
/// @file DebouncedDigitalInput.h
///
/// @brief Abstracted digital input with debouncing applied
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

#include "DebouncedDigitalInput.h"
#include <Arduino.h>

DebouncedDigitalInput::DebouncedDigitalInput(uint8_t pinNumber, bool usePullup)
    : DebouncedDigitalInput(pinNumber, NOT_A_PIN, usePullup) {
  // Do nothing
}

DebouncedDigitalInput::DebouncedDigitalInput(uint8_t pinNumber, uint8_t groundPin, bool usePullup)
    : DebouncedDigitalInput(pinNumber, groundPin, 0, usePullup) {
  // Do nothing
}

DebouncedDigitalInput::DebouncedDigitalInput(uint8_t pinNumber,
                                             uint8_t groundPin,
                                             uint16_t debounceSamples,
                                             bool usePullup)
    : m_readPin(pinNumber)
    , m_groundPin(groundPin)
    , m_usePullup(usePullup)
    , m_debounceSamples(debounceSamples)
    , m_initialized(false)
    , m_debounceCount(0)
    , m_rawValue(false)
    , m_debouncedValue(false) {
  // Do nothing
}

void DebouncedDigitalInput::Initialize() {
  if (!m_initialized) {
    if (NOT_A_PIN != m_readPin) {
      pinMode(m_readPin, m_usePullup ? INPUT_PULLUP : INPUT);
    }
    if (NOT_A_PIN != m_groundPin) {
      pinMode(m_groundPin, OUTPUT);
      digitalWrite(m_groundPin, LOW);
    }
  }
  m_initialized = true;
}

bool DebouncedDigitalInput::Update() {
  if (false == m_initialized) {
    return false;
  }

  m_rawValue = (NOT_A_PIN != m_readPin ? (digitalRead(m_readPin) == HIGH) : false);
  m_rawValue = m_usePullup ? !m_rawValue : m_rawValue;
  if (m_rawValue != m_debouncedValue) {
    ++m_debounceCount;
    if (m_debounceCount >= m_debounceSamples) {
      m_debouncedValue = m_rawValue;
      m_debounceCount = 0;
    }
  } else {
    m_debounceCount = 0;
  }

  return m_debouncedValue;
}

bool DebouncedDigitalInput::GetValue() const {
  return m_debouncedValue;
}

bool DebouncedDigitalInput::GetRawValue() const {
  return m_rawValue;
}

void DebouncedDigitalInput::Reset(bool newValue) {
  m_rawValue = newValue;
  m_debouncedValue = newValue;
  m_debounceCount = 0;
}

void DebouncedDigitalInput::SetDebounceCount(uint16_t newCount) {
  m_debounceSamples = newCount;
  if (m_debounceSamples <= m_debounceCount) {
    m_debouncedValue = m_rawValue;
    m_debounceCount = 0;
  }
}
