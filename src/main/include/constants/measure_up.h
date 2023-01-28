/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/length.h>

#include <array>

namespace measure_up {
  constexpr auto bumperExtension = 3_in;  ///< Distance from frame to outer edge of bumpers
  namespace chassis {
    constexpr units::inch_t width{26.0};
    constexpr units::inch_t length{33.0};
  }  // namespace chassis
  namespace swerve_offsets {
    constexpr auto frontLeftLOffset = 2.625_in;
    constexpr auto frontLeftWOffset = 2.625_in;
    constexpr auto frontRightLOffset = 2.625_in;
    constexpr auto frontRightWOffset = 2.625_in;
    constexpr auto backRightWOffset = 2.625_in;
    constexpr auto backRightLOffset = 2.625_in;
    constexpr auto backLeftWOffset = 2.625_in;
    constexpr auto backLeftLOffset = 2.625_in;
  }  // namespace swerve_offsets
  namespace arm_extension {
    constexpr auto homeExtension = 35.25_in;
    constexpr auto maxExtension = 35.00_in;
    constexpr auto minExtension = 1_in;

  }  // namespace arm_extension
}  // namespace measure_up
