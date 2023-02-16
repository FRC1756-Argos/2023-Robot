////////////////////////////////////////////////////////////////////////////////
/// @file DebouncedButtonSet.h
///
/// @brief Manages a set of debounced inputs
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

#ifndef DebouncedButtonSet_H
#define DebouncedButtonSet_H

#include <stdint.h>
#include "DebouncedDigitalInput.h"

template <size_t N>
class DebouncedButtonSet {
 public:
  DebouncedButtonSet() = delete;
  DebouncedButtonSet(DebouncedDigitalInput (&&buttons)[N])
      : m_buttonSet(buttons), m_initialized(false), m_numButtons(N){};

  DebouncedButtonSet(const DebouncedButtonSet&) = delete;

  DebouncedButtonSet& operator=(const DebouncedButtonSet&) = delete;

  void Initialize() {
    if (!m_initialized) {
      for (size_t i = 0; i < N; ++i) {
        m_buttonSet[i].Initialize();
      }
    }
    m_initialized = true;
  };

  void Update() {
    if (m_initialized) {
      for (size_t i = 0; i < N; ++i) {
        m_buttonSet[i].Update();
      }
    }
  };

  bool GetValue(size_t idx) const {
    bool retVal = false;
    if (m_initialized && idx < N) {
      retVal = m_buttonSet[idx].GetValue();
    }
    return retVal;
  };

  bool GetRawValue(size_t idx) const {
    bool retVal = false;
    if (m_initialized && idx < N) {
      retVal = m_buttonSet[idx].GetRawValue();
    }
    return retVal;
  };

  void Reset(bool newValue = false) {
    if (m_initialized) {
      for (size_t i = 0; i < N; ++i) {
        m_buttonSet[i].Reset(newValue);
      }
    }
  };
  void Reset(size_t idx, bool newValue = false) {
    if (m_initialized && idx < N) {
      m_buttonSet[idx].Reset(newValue);
    }
  };

  void SetDebounceCount(size_t idx, uint16_t newCount) {
    if (m_initialized && idx < N) {
      m_buttonSet[idx].SetDebounceCount(newCount);
    }
  };
  void SetDebounceCount(uint16_t newCount) {
    if (m_initialized) {
      for (size_t i = 0; i < N; ++i) {
        m_buttonSet[i].SetDebounceCount(newCount);
      }
    }
  };

  size_t GetNumButtons() const { return m_numButtons; };

 private:
  DebouncedDigitalInput m_buttonSet[N];
  bool m_initialized;
  const size_t m_numButtons;
};

#endif
