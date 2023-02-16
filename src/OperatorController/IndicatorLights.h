////////////////////////////////////////////////////////////////////////////////
/// @file IndicatorLights.h
///
/// @brief Manages a set of indicator lights
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

#ifndef IndicatorLights_H
#define IndicatorLights_H

#include "LED.h"

template<size_t N>
class IndicatorLights
{
  public:
    IndicatorLights() = delete;
    IndicatorLights( LED (&&lights)[N],
                     size_t maxIlluminated = N ): m_lights(lights),
                                                  m_initialized(false),
                                                  m_numLEDs(N),
                                                  m_maxIlluminated(maxIlluminated),
                                                  m_numIlluminated(0) {};

    void Initialize()
    {
      if(!m_initialized)
      {
        for(size_t i = 0; i < N; ++i)
        {
          m_lights[i].Initialize();
        }
        m_initialized = true;
      }
    };

    void Update()
    {
      if(m_initialized)
      {
        for(size_t i = 0; i < N; ++i)
        {
          m_lights[i].Update();
        }
      }
    };

    void On(size_t idx)
    {
      if(m_initialized && idx < m_numLEDs)
      {
        bool activeIllumination = m_lights[idx].GetIlluminated();
        if(!activeIllumination && m_numIlluminated < m_maxIlluminated)
        {
          ++m_numIlluminated;
          m_lights[idx].On();
        }
      }
    };

    void Off(size_t idx)
    {
      if(m_initialized && idx < m_numLEDs)
      {
        bool activeIllumination = m_lights[idx].GetIlluminated();
        if(activeIllumination)
        {
          --m_numIlluminated;
          m_lights[idx].Off();
        }
      }
    };

    void Set(size_t idx, bool val)
    {
      if(m_initialized)
      {
        if(val)
        {
          On(idx);
        }
        else
        {
          Off(idx);
        }
      }
    }

    LED::LEDPattern GetPattern(size_t idx) const
    {
      LED::LEDPattern retVal = LED::LEDPattern::SOLID;
      if(m_initialized && idx < m_numLEDs)
      {
        retVal = m_lights[idx].GetPattern();
      }
      return retVal;
    };

    void SetPattern(size_t idx, LED::LEDPattern newPattern)
    {
      if(m_initialized && idx < m_numLEDs)
      {
        m_lights[idx].SetPattern(newPattern);
      }
    };

    size_t GetNumLights() const
    {
      return m_numLEDs;
    };

    bool GetIlluminated(size_t idx) const
    {
      bool retVal = false;
      if(m_initialized && idx < m_numLEDs)
      {
        retVal = m_lights[idx].GetIlluminated();
      }
      return retVal;
    };

  private:
    LED                   m_lights[N];
    bool                  m_initialized;
    const size_t          m_numLEDs;
    const size_t          m_maxIlluminated;
    size_t                m_numIlluminated;
};

#endif
