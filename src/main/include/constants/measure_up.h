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
  namespace lifter {
    namespace arm_extension {
      constexpr auto homeExtension = 37.50_in;
      constexpr auto maxExtension = 74.00_in;
      constexpr auto minExtension = 38.00_in;

    }  // namespace arm_extension
    namespace wrist {
      constexpr auto homeAngle = 0_deg;
      constexpr auto minAngle = -180_deg;
      constexpr auto maxAngle = 180_deg;
    }  // namespace wrist
    namespace shoulder {
      constexpr auto homeAngle = 180_deg;
      constexpr auto minAngle = 135_deg;
      constexpr auto maxAngle = 225_deg;
    }  // namespace shoulder
  }    // namespace lifter
}  // namespace measure_up
