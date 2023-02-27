/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "utils/custom_units.h"

struct ArmState {
  units::meter_t armLen;
  units::radian_t shoulderAngle;
};

class LifterKinematics {
 public:
  /// @brief Construct lifter kinematics object
  /// @param fulcrumPosition Fulcrum position in robot coordinate space (y is actually z)
  /// @param armRotationOffset Offset from center of lifter rotation axis to measure up point at
  ///                          end of arm in radial direction
  /// @param effectorOffset End effector offset from measure up point at end of arm.
  ///                       y is actually z.  Measured when arm is at 0 deg
  LifterKinematics(const frc::Translation2d& fulcrumPosition,
                   const units::meter_t armRotationOffset,
                   const frc::Translation2d& effectorOffset,
                   const frc::Translation2d& fixedBoomActuatorPosition,
                   const frc::Translation2d& actuatedBoomActuatorPosition);

  LifterKinematics() = delete;

  /// @brief Solves for the lifter joint from a desired effector state
  /// @param pose Desired effector position in robot coordinate space
  /// @param effectorInverted True indicates the wrist is rotated so end effector offsets are opposite y
  /// @return LifterState object describing the state of the joint to get to desired point
  ArmState GetJoints(frc::Translation2d pose, bool effectorInverted = false) const;

  /// @brief Solves for effector position based off of the lifter state
  /// @param state ArmState struct containing current lifter state
  /// @param effectorInverted True indicates the wrist is rotated so end effector offsets are opposite y
  /// @return Translation2d object describing position of effector in robot coordinate space
  frc::Translation2d GetPose(ArmState state, bool effectorInverted = false) const;

  units::inch_t ShoulderAngleToBoomExtension(units::degree_t shoulderAngle) const;
  units::degree_t BoomExtensionToShoulderAngle(units::inch_t boomExtension) const;
  units::velocity::inches_per_second_t ShoulderVelocityToBoomVelocity(units::radians_per_second_t shoulderVelocity,
                                                                      units::degree_t shoulderAngle) const;
  units::radians_per_second_t BoomVelocityToShoulderVelocity(units::velocity::inches_per_second_t boomVelocity,
                                                             units::inch_t boomPosition) const;

 private:
  const frc::Translation2d m_fulcrumPosition;  ///< Fulcrum position in robot coordinate space (y is actually z)
  const units::meter_t
      m_armRotationOffset;  ///< Offset from center of lifter rotation axis to measure up point at end of arm in radial direction
  const frc::Translation2d
      m_effectorOffset;  ///< End effector offset from measure up point at end of arm.  y is actually z.  Measured when arm is at 0 deg

  const units::inch_t m_fixedBoomAnchorToFulcrumDist;
  const units::degree_t m_fixedBoomAnchorToFulcrumAngle;
  const units::inch_t m_articulatedBoomAnchorToFulcrumDist;
  const units::degree_t m_articulatedBoomAnchorToFulcrumAngle;
};
