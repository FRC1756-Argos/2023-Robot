/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"

namespace controlLoop {
  namespace comp_bot {
    namespace drive {
      struct rotate {
        constexpr static double kP = 1.4;
        constexpr static double kI = 0.0005;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.0;
        constexpr static double iZone = 500.0;
        constexpr static double allowableError = 0.0;
      };  // namespace rotate
      struct drive {
        constexpr static double kP = 0.11;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.05;
        constexpr static double iZone = 500.0;
        constexpr static double allowableError = 0.0;
      };  // namespace drive
      struct linear_follower {
        constexpr static double kP = 8.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
      };  // namespace linear_follower
      struct rotational_follower {
        constexpr static double kP = 8.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static auto angularVelocity = units::degrees_per_second_t{360};
        constexpr static auto angularAcceleration = units::degrees_per_second_squared_t{360};
      };  // namespace rotational_follower
    }     // namespace drive
    namespace lifter {
      struct armExtension {
        constexpr static double kP = 0.2;
        constexpr static double kI = 0;
        constexpr static double kD = 0;
        constexpr static double kF = 0;
        constexpr static double iZone = 0;
        constexpr static double allowableError = 0;
      };
      struct shoulder {
        constexpr static double kP = 0;
        constexpr static double kI = 0;
        constexpr static double kD = 0;
        constexpr static double kF = 0;
        constexpr static double iZone = 0;
        constexpr static double allowableError = 0;
      };
      struct wrist {
        constexpr static double kP = 0;
        constexpr static double kI = 0;
        constexpr static double kD = 0;
        constexpr static double kF = 0;
        constexpr static double iZone = 0;
        constexpr static double allowableError = 0;
      };
    }  // namespace lifter
  }    // namespace comp_bot
  namespace practice_bot {
    namespace drive {
      using rotate = controlLoop::comp_bot::drive::rotate;
      using drive = controlLoop::comp_bot::drive::drive;
      using linear_follower = controlLoop::comp_bot::drive::linear_follower;
      using rotational_follower = controlLoop::comp_bot::drive::rotational_follower;
    }  // namespace drive
    namespace lifter {
      using armExtension = comp_bot::lifter::armExtension;
      using shoulder = comp_bot::lifter::shoulder;
      using write = comp_bot::lifter::wrist;
    }  // namespace lifter
  }    // namespace practice_bot
}  // namespace controlLoop
