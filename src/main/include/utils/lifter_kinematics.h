/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>

struct LifterState {
  units::meter_t armLen;
  units::radian_t shoulderAngle;
};

class LifterKinematics {
 public:
  LifterKinematics(const frc::Translation2d& fulcrumPosition,
                   const units::meter_t armRotationOffset,
                   const frc::Translation2d& effectorOffset);

  /// @brief Solves for the lifter joint from a desired effector state
  /// @param pose Desired effector position in lifter coordinate space
  /// @param effectorInverted True indicates the wrist is rotated so end effector offsets are opposite y
  /// @return LifterState object describing the state of the joint to get to desired point
  LifterState GetJoints(frc::Translation2d pose, bool effectorInverted = false);

  /// @brief Solves for effector position based off of the lifter state
  /// @param state LifterState struct containing current lifter state
  /// @param effectorInverted True indicates the wrist is rotated so end effector offsets are opposite y
  /// @return Translation2d object describing position of effector in lifter coordinate space
  frc::Translation2d GetPose(LifterState state, bool effectorInverted = false);

 private:
  const frc::Translation2d m_fulcrumPosition;
  const units::meter_t m_armRotationOffset;
  const frc::Translation2d& m_effectorOffset;
};
