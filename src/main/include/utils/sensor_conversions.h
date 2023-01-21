/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include "units/acceleration.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"

namespace units {
  UNIT_ADD(velocity,
           inches_per_second,
           inches_per_second,
           ips,
           units::compound_unit<units::length::inches, units::inverse<units::time::second>>)
  UNIT_ADD(acceleration,
           inches_per_second_squared,
           inches_per_second_squared,
           ips2,
           units::compound_unit<units::length::inches, units::inverse<units::squared<units::time::second>>>)
}  // namespace units

namespace sensor_conversions {
  namespace swerve_drive {
    namespace turn {
      constexpr double sensorConversionFactor =
          360.0 / 4096;  ///< multiply to convert raw sensor units to module degrees
      constexpr double ToSensorUnit(const units::degree_t degrees) {
        return degrees.to<double>() / sensorConversionFactor;
      }
      constexpr units::degree_t ToAngle(const double sensorunit) {
        return units::make_unit<units::degree_t>(sensorunit * sensorConversionFactor);
      }
    }  // namespace turn
    namespace drive {
      constexpr auto wheelDiameter = 4_in;
      constexpr auto wheelCircumference = wheelDiameter * M_PI;
      constexpr double sensorUnitsPerMotorRevolution = 2048;
      constexpr double driveGearRatio = 8.16;

      constexpr units::inch_t ToDistance(const double sensorunit) {
        return wheelCircumference * (sensorunit / sensorUnitsPerMotorRevolution / driveGearRatio);
      }
      constexpr double ToSensorPosition(const units::inch_t distance) {
        return (distance / wheelCircumference) * driveGearRatio * sensorUnitsPerMotorRevolution;
      }

      constexpr units::inches_per_second_t ToVelocity(const double sensorVelocity) {
        return ToDistance(sensorVelocity) / units::decisecond_t{1};
      }
      constexpr double ToSensorVelocity(const units::inches_per_second_t velocity) {
        return ToSensorPosition(velocity * units::decisecond_t{1});
      }
    }  // namespace drive
    namespace lifter {
      namespace armExtension {
        constexpr double sensorToMotorRevolution = 1.0 / 2048;
        constexpr double gearboxReduction = 1.0 / 12;
        constexpr double driveSprocketTeeth = 15.0;
        constexpr double extensionInchesPerTooth = 0.325 / 1;

        constexpr units::inch_t ToExtension(const double sensorUnit) {
          return units::make_unit<units::inch_t>(sensorUnit * sensorToMotorRevolution * gearboxReduction *
                                                 driveSprocketTeeth * extensionInchesPerTooth);
        }

        constexpr double ToSensorUnit(const units::inch_t extension) {
          return extension.to<double>() / driveSprocketTeeth / gearboxReduction / sensorToMotorRevolution /
                 sensorToMotorRevolution;
        }
      }  // namespace armExtension
    }    // namespace lifter
  }      // namespace swerve_drive
}  // namespace sensor_conversions
