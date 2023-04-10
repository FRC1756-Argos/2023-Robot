/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/PneumaticsModuleType.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <string>

#include "constants/addresses.h"
#include "constants/control_loops.h"
#include "constants/interpolation_maps.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

namespace speeds {
  namespace drive {
    constexpr double aimBotMaxBias = 0.6;
    constexpr double aimBotThresh = 0.1;
    constexpr units::velocity::feet_per_second_t maxAngular = 12_fps;
  }  // namespace drive
  namespace armKinematicSpeeds {
    constexpr auto effectorVelocity = 60_ips;
    constexpr auto effectorAcceleration = 45_ips2;
    constexpr auto effectorFastVelocity = 80_ips;
    constexpr auto effectorFastAcceleration = 60_ips2;
  }  // namespace armKinematicSpeeds
}  // namespace speeds

namespace indexes {
  namespace swerveModules {
    constexpr char frontLeftIndex = 0;
    constexpr char frontRightIndex = 1;
    constexpr char backRightIndex = 2;
    constexpr char backLeftIndex = 3;
  }  // namespace swerveModules
}  // namespace indexes

namespace paths {
  const std::string swerveHomesPath = "homes/swerveHomes";
  const std::string wristHomesPath = "homes/wristHomes";
  const std::string shoulderHome = "homes/shoulderHome";
  const std::string extensionHomePath = "homes/extensionHome";
}  // namespace paths

namespace networkTables {
  namespace swerveHomes {
    const std::string tableKey = "Argos";
    namespace keys {
      const std::string flHome = "swerveHomes/flHome";
      const std::string frHome = "swerveHomes/frHome";
      const std::string brHome = "swerveHomes/brHome";
      const std::string blHome = "swerveHomes/blHome";

      const std::string flHomeFullPath = "swerveHomes/flHome";
      const std::string frHomeFullPath = "swerveHomes/frHome";
      const std::string brHomeFullPath = "swerveHomes/brHome";
      const std::string blHomeFullPath = "swerveHomes/blHome";
    }  // namespace keys
  }    // namespace swerveHomes
}  // namespace networkTables

namespace leds {
  // Length of request animation in seconds
  constexpr units::time::second_t requestLen = 2_s;
}  // namespace leds
namespace camera {
  constexpr char reflectivePipeline = 1;
  constexpr char aprilTagPipeline = 0;
  constexpr int horizontalPixelResolution = 320;
  constexpr int verticalPixelResolution = 240;
  constexpr auto horizontalAngleResolution = 63.3_deg;
  constexpr auto halfhorizontalAngleResolution = horizontalAngleResolution / 2;
  constexpr auto verticalAngleResolution = 49.7_deg;
}  // namespace camera

/// @brief Designate the threshold approach direction
enum class ApproachDirection { Increasing, Decreasing };

/// @brief This is a bad idea...
namespace warning {
  /// @brief Are you sure you want to change this?!?!
  namespace nuclear {
    /// @brief Turn back now!
    namespace option {
      constexpr bool wristEnabled = true;  ///< @warning Only set to false if wrist is broken beyond repair :(
    }                                      // namespace option
  }                                        // namespace nuclear
}  // namespace warning
