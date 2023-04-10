/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <numbers>

#include "custom_units.h"

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
      constexpr auto wheelDiameter = 3.5_in;
      constexpr auto wheelCircumference = wheelDiameter * std::numbers::pi;
      constexpr double sensorUnitsPerMotorRevolution = 2048;
      constexpr double driveGearRatio = 36000.0 / 5880.0;

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
  }    // namespace swerve_drive

  namespace lifter {
    namespace arm_extension {
      constexpr double sensorToMotorRevolution = 1.0 / 2048;
      constexpr double gearboxReduction = 1.0 / 12.0;
      constexpr double driveSprocketTeeth = 15.0;
      constexpr double extensionInchesPerTooth = 0.375 / 1;
      constexpr double absEncoderReduction = 1.0 / 10;
      constexpr units::inch_t extensionPerEncoderRevolution =
          units::make_unit<units::inch_t>((extensionInchesPerTooth * driveSprocketTeeth) / absEncoderReduction);

      constexpr units::inch_t ToExtension(const double sensorUnit) {
        return units::make_unit<units::inch_t>(sensorUnit * sensorToMotorRevolution * gearboxReduction *
                                               driveSprocketTeeth * extensionInchesPerTooth);
      }
      constexpr double ToSensorUnit(const units::inch_t extension) {
        return extension.to<double>() / sensorToMotorRevolution / gearboxReduction / driveSprocketTeeth /
               extensionInchesPerTooth;
      }

      constexpr units::inches_per_second_t ToVelocity(const double sensorVelocity) {
        return units::inches_per_second_t{ToExtension(sensorVelocity) / units::decisecond_t{1}};
      }
      constexpr double ToSensorVelocity(const units::inches_per_second_t velocity) {
        return ToSensorUnit(velocity * units::decisecond_t{1});
      }

      // * David I'm sorry in advance for this math

      /// @brief Find the travel of the extension given a rotation "angle" of the CANCoder
      /// @param angle Rotation of the CANCoder
      /// @return The travel of extension given a rotation of geared CANCoder
      constexpr units::inch_t ExtensionFromRotation(const units::degree_t angle) {
        return extensionPerEncoderRevolution *
               units::make_unit<units::dimensionless::scalar_t>((angle.to<double>() / 360.0));
      }
    }  // namespace arm_extension
    namespace wrist {
      constexpr double sensorUnitsPerRevolution = 2048.0;
      constexpr double gearBoxReduction = 9.0;
      constexpr double extraReduction = 60.0 / 20.0;
      constexpr double sensorConversionFactor =
          360.0 / (sensorUnitsPerRevolution * gearBoxReduction *
                   extraReduction);  // Scalar for getting angle in degrees from encoder
      constexpr units::degree_t ToAngle(const double sensorUnit) {
        return units::make_unit<units::degree_t>(sensorUnit * sensorConversionFactor);
      }

      constexpr double ToSensorUnit(const units::degree_t degrees) {
        return degrees.to<double>() / sensorConversionFactor;
      }

      constexpr units::degrees_per_second_t ToVelocity(const double sensorVelocity) {
        return units::degrees_per_second_t{ToAngle(sensorVelocity) / units::decisecond_t{1}};
      }
      constexpr double ToSensorVelocity(const units::degrees_per_second_t velocity) {
        return ToSensorUnit(velocity * units::decisecond_t{1});
      }

    }  // namespace wrist
    namespace shoulder {
      constexpr double sensorConversionFactor =
          360.0 / 4096;  ///< multiply to convert raw sensor units to module degrees
      constexpr double ToSensorUnit(const units::degree_t degrees) {
        return degrees.to<double>() / sensorConversionFactor;
      }
      constexpr units::degree_t ToAngle(const double sensorUnit) {
        return units::degree_t(sensorUnit * sensorConversionFactor);
      }

      constexpr units::degrees_per_second_t ToVelocity(const double sensorVelocity) {
        return units::degrees_per_second_t{ToAngle(sensorVelocity) / units::decisecond_t{1}};
      }
      constexpr double ToSensorVelocity(const units::degrees_per_second_t velocity) {
        return ToSensorUnit(velocity * units::decisecond_t{1});
      }
    }  // namespace shoulder
    namespace shoulder_actuator {
      constexpr double sensorToMotorRev = 1.0 / 2048;
      constexpr double beltReduction = 30.0 / 18.0;
      constexpr double extensionMillimetersPerRevolution = 4.0;
      constexpr double fudgeFactor = 0.992;  ///< This seems to be due to some discrepancy on comp bot

      constexpr double ToSensorUnit(const units::inch_t extension) {
        return (units::millimeter_t(extension) / extensionMillimetersPerRevolution / beltReduction / sensorToMotorRev /
                fudgeFactor)
            .to<double>();
      }

      constexpr units::inch_t ToExtension(const double sensorUnits) {
        return units::millimeter_t(sensorUnits * sensorToMotorRev * beltReduction * extensionMillimetersPerRevolution *
                                   fudgeFactor);
      }

      constexpr units::inches_per_second_t ToVelocity(const double sensorVelocity) {
        return units::inches_per_second_t{ToExtension(sensorVelocity) / units::decisecond_t{1}};
      }
      constexpr double ToSensorVelocity(const units::inches_per_second_t velocity) {
        return ToSensorUnit(velocity * units::decisecond_t{1});
      }
    }  // namespace shoulder_actuator
  }    // namespace lifter
  namespace bashguard {
    constexpr double sensorToMotorRevolution = 1.0 / 2048;
    constexpr double gearboxReduction = 1.0 / 5;
    constexpr double driveSprocketTeeth = 15.0;
    constexpr double extensionInchesPerTooth = 0.375 / 1;
    constexpr units::inch_t ToExtension(const double sensorUnit) {
      return units::make_unit<units::inch_t>(sensorUnit * sensorToMotorRevolution * gearboxReduction *
                                             driveSprocketTeeth * extensionInchesPerTooth);
    }
    constexpr double ToSensorUnit(const units::inch_t extension) {
      return extension.to<double>() / sensorToMotorRevolution / gearboxReduction / driveSprocketTeeth /
             extensionInchesPerTooth;
    }

    constexpr units::inches_per_second_t ToVelocity(const double sensorVelocity) {
      return units::inches_per_second_t{ToExtension(sensorVelocity) / units::decisecond_t{1}};
    }
    constexpr double ToSensorVelocity(const units::inches_per_second_t velocity) {
      return ToSensorUnit(velocity * units::decisecond_t{1});
    }
  }  // namespace bashguard

  namespace oui_oui_place {

    constexpr double sensorConversionFactor = 360.0 / 2048;  ///< multiply to convert raw sensor units to module degrees
    constexpr double gearboxReduction = 1.0 / 15.0;

    constexpr double ToSensorUnit(const units::degree_t angle) {
      return angle.to<double>() / sensorConversionFactor / gearboxReduction;
    }

    constexpr units::degree_t ToAngle(const double sensorUnits) {
      return units::degree_t(sensorUnits * sensorConversionFactor * gearboxReduction);
    }

    constexpr units::degrees_per_second_t ToVelocity(const double sensorVelocity) {
      return units::degrees_per_second_t{ToAngle(sensorVelocity) / units::decisecond_t{1}};
    }

    constexpr double ToSensorVelocity(const units::degrees_per_second_t velocity) {
      return ToSensorUnit(velocity * units::decisecond_t{1});
    }
  }  // namespace oui_oui_place
}  // namespace sensor_conversions
