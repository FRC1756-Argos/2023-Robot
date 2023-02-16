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

#ifndef DebouncedDigitalInput_H
#define DebouncedDigitalInput_H

#include <stdint.h>
#include "MiscConstants.h"

class DebouncedDigitalInput
{
  public:
    DebouncedDigitalInput() = delete;
    DebouncedDigitalInput( uint8_t pinNumber,
                           bool    usePullup = false );
    DebouncedDigitalInput( uint8_t pinNumber,
                           uint8_t groundPin,
                           bool    usePullup = false );
    DebouncedDigitalInput( uint8_t  pinNumber,
                           uint8_t  groundPin,
                           uint16_t debounceSamples,
                           bool     usePullup = false );

    DebouncedDigitalInput& operator=(const DebouncedDigitalInput&) = delete;

    void Initialize();

    bool Update();

    bool GetValue() const;

    bool GetRawValue() const;

    void Reset(bool newValue = false);

    void SetDebounceCount(uint16_t newCount);

  private:
    const uint8_t m_readPin;         ///< Arduino pin number for this input
    const uint8_t m_groundPin;       ///< Arduino pin number used as a ground source
    const bool    m_usePullup;       ///< True uses Arduino internal pullup resistor
    uint16_t      m_debounceSamples; ///< Number of samples new value must be held before debounced value changes
    bool          m_initialized;     ///< True once initialization has completed
    uint16_t      m_debounceCount;   ///< Number of samples that do not match the current debounced value
    bool          m_rawValue;        ///< Latest read value without debouncing
    bool          m_debouncedValue;  ///< Active debounced value
};

#endif
