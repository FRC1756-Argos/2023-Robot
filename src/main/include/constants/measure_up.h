/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/length.h>

#include <array>

#include "utils/path_planning/types.h"

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
    // Y is actually Z
    constexpr auto fulcrumPosition = frc::Translation2d{-12_in, 50_in};
    namespace arm_extension {
      constexpr auto homeExtension = 37.50_in;
      constexpr auto maxExtension = 68.00_in;
      constexpr auto minExtension = 38.00_in;
      constexpr auto acceptErr = 0.5_in;

    }  // namespace arm_extension
    namespace wrist {
      constexpr auto homeAngle = 0_deg;
      constexpr auto minAngle = -180_deg;
      constexpr auto invertedAngle = -180_deg;
      constexpr auto nominalAngle = 0_deg;
      constexpr auto maxAngle = 10_deg;
      constexpr auto wristWidth = 18.5_in;
    }  // namespace wrist
    namespace shoulder {
      constexpr auto homeAngle = 0_deg;
      constexpr auto minAngle = -59_deg;
      constexpr auto maxAngle = 5_deg;
      constexpr auto acceptErr = 2_deg;
      constexpr auto fixedBoomActuatorPosition =
          frc::Translation2d{-14.75_in, 4.5_in};  ///< Linear actuator mount point relative to robot origin
      constexpr auto actuatedBoomActuatorPosition =
          frc::Translation2d{14_in, -2.5_in};  ///< Linear actuator mount point relative to fulcrum.
      /// x is along length of arm, y is up with zero at same elevation as fulcrum
      /// (when arm in front of robot parallel to ground)
    }  // namespace shoulder
    namespace armBar {
      constexpr auto centerOfRotDis = 1.35_in;
    }  // namespace armBar
    namespace effector {
      // Y is actually Z
      constexpr auto effectorFromArm = frc::Translation2d{3.5_in, 1.5_in};
    }  // namespace effector
  }    // namespace lifter
  namespace bash {
    constexpr auto homeExtension = 4.25_in;
    constexpr auto retractedExtension = 4.5_in;
    constexpr auto deployedExtension = 24.5_in;
    constexpr auto minExtension = 4.5_in;
    constexpr auto maxExtension = 25_in;
  }  // namespace bash
  // namespace wrist {
  //   constexpr auto
  constexpr std::array<path_planning::ArmPathPoint, 13> PathPlanningKeepOutZone = {
      path_planning::ArmPathPoint{-chassis::length / 2 - bumperExtension, 0_in},
      path_planning::ArmPathPoint{-chassis::length / 2 - 48_in, 0_in},
      path_planning::ArmPathPoint{-chassis::length / 2 - 48_in, 78_in},
      path_planning::ArmPathPoint{chassis::length / 2 + 48_in, 78_in},
      path_planning::ArmPathPoint{chassis::length / 2 + 48_in, 0_in},
      path_planning::ArmPathPoint{chassis::length / 2 + bumperExtension, 0_in},
      path_planning::ArmPathPoint{chassis::length / 2 + bumperExtension, 8_in},
      // path_planning::ArmPathPoint{chassis::length / 2, 18_in},
      // path_planning::ArmPathPoint{chassis::length / 2, 12_in},
      path_planning::ArmPathPoint{0_in, 8_in},
      path_planning::ArmPathPoint{lifter::fulcrumPosition.X(), lifter::fulcrumPosition.Y()},
      path_planning::ArmPathPoint{-chassis::length / 2, 8_in},
      path_planning::ArmPathPoint{-chassis::length / 2 - bumperExtension, 8_in}};
}  // namespace measure_up
