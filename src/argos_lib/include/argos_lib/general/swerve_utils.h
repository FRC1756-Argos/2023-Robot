/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <optional>

#include "interpolation.h"

namespace argos_lib {
  namespace swerve {

    /**
     * @brief Finds closest angle alias of desiredAngle relative to referencedAngle.
     *        All inputs are normalized so ranges are unbounded.
     *
     * @param desiredAngle Angle to find alias for
     * @param referenceAngle Defines desired alias range
     * @return Angle alias of desiredAngle in range (referencedAngle - 180_deg, referencedAngle + 180_deg]
     */
    units::degree_t NearestAngle(units::degree_t desiredAngle, units::degree_t referenceAngle);

    /**
     * @brief Finds closest angle alias of a vector 180-degrees offset from desiredAngle relative to referencedAngle.
     *        All inputs are normalized so ranges are unbounded.
     *
     * @param desiredAngle Angle to find 180-degree offset alias for
     * @param referenceAngle Defines desired alias range
     * @return Offset angle alias of desiredAngle in range (referencedAngle - 180_deg, referencedAngle + 180_deg]
     */
    units::degree_t InvertedAngle(units::degree_t desiredAngle, units::degree_t referenceAngle);

    /**
     * @brief Normalize angle to specified range
     *
     * @param inVal Angle to constrain
     * @param minVal Normalization lower bound
     * @param maxVal Normalization upper bound
     * @return Normalized value in range [minVal, maxVal]
     */
    units::degree_t ConstrainAngle(units::degree_t inVal, units::degree_t minVal, units::degree_t maxVal);
    /**
     * @brief Normalize angle to specified range
     *
     * @param inVal Angle to constrain (degrees)
     * @param minVal Normalization lower bound (degrees)
     * @param maxVal Normalization upper bound (degrees)
     * @return Normalized value in range [minVal, maxVal] (degrees)
     */
    double ConstrainAngle(double inVal, double minVal, double maxVal);

    /**
     * @brief Optimize swerve module to minimize rotations and drive direction changes
     *
     * @param desiredState Requested state.  Output must result in same motion
     * @param currentModuleAngle Module rotation angle in relative or absolute position
     * @param currentModuleAngularRate Current module rotation speed.  To prevent rapid changes in rotation direction.
     * @param currentModuleDriveVel Current module drive velocity.  To prevent rapid changes in drive motor velocity.
     * @param maxVelocity Max velocity for determining max transition change thresholds
     * @return Optimized swerve module state that results in same motion as desiredState
     */
    frc::SwerveModuleState Optimize(frc::SwerveModuleState desiredState,
                                    units::degree_t currentModuleAngle,
                                    units::degrees_per_second_t currentModuleAngularRate,
                                    units::feet_per_second_t currentModuleDriveVel,
                                    units::feet_per_second_t maxVelocity);

    /**
     * @brief Representation of the absolute encoder position of each module at home position
     */
    struct SwerveModulePositions {
      units::degree_t FrontLeft;
      units::degree_t FrontRight;
      units::degree_t RearRight;
      units::degree_t RearLeft;
    };

    /**
     * @brief Translation speeds as percent max output
     */
    struct TranslationSpeeds {
      double forwardSpeedPct;  ///< Speed with positive forward in range [-1, 1]
      double leftSpeedPct;     ///< Speed with positive left in range [-1, 1]
    };

    /**
     * @brief Use argos_lib::InterpolationMap to apply mapping according to joystick vector magnitude
     *
     * @tparam T Type of interpolated input
     * @tparam size Number of elements in interpolation map
     * @tparam V Type of interpolated output
     * @param rawSpeeds Joystick inputs as percentages
     * @param interpMap Interpolation map to apply to magnitudes
     * @return Remapped joystick percentages.  Angle of vector will match rawSpeeds, but magnitude will
     *         scale according to interpMap.  This results in circular deadband and other mapping results.
     */
    template <class T, int size, class V>
    [[nodiscard]] constexpr TranslationSpeeds CircularInterpolate(
        const TranslationSpeeds rawSpeeds, const argos_lib::InterpolationMap<T, size, V> interpMap) {
      const double magnitude = std::sqrt(std::pow(rawSpeeds.forwardSpeedPct, 2) + std::pow(rawSpeeds.leftSpeedPct, 2));
      const double angle = std::atan2(rawSpeeds.leftSpeedPct, rawSpeeds.forwardSpeedPct);
      const double mappedMagnitude = interpMap(magnitude);
      return TranslationSpeeds{mappedMagnitude * std::cos(angle), mappedMagnitude * std::sin(angle)};
    }

    /**
     * @brief Interface capable of saving and loading module home positions from persistent storage
     */
    class SwerveHomeStorageInterface {
     public:
      /**
       * @brief Save home position to persistent storage
       *
       * @param homePosition Positions to store
       * @return true Save successful
       * @return false Error saving
       */
      virtual bool Save(const SwerveModulePositions& homePosition) = 0;

      /**
       * @brief Load home position from persistent storage
       *
       * @return Poisitions from persistent storage or std::nullopt if load failed or no positions were
       *         previously stored
       */
      [[nodiscard]] virtual std::optional<SwerveModulePositions> Load() = 0;
    };

  }  // namespace swerve
}  // namespace argos_lib
