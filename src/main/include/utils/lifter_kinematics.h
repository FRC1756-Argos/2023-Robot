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
  LifterKinematics();

  /// @brief Solves for the lifter joint from a desired effector state
  /// @param state Desired effector position in lifter coordinate space
  /// @param effectorOffset Offset of effector from rotation center in lifter coordinate space
  /// @return LifterState object describing the state of the joint to get to desired point
  static LifterState GetJoints(frc::Translation2d pose, units::meter_t effectorOffset);

  /// @brief Solves for effector position based off of the lifter state
  /// @param state LifterState struct containing current lifter state
  /// @param effectorOffset Offset of effector from rotation center in lifter coordinate space
  /// @return Translation2d object describing position of effector in lifter coordinate space
  static frc::Translation2d GetPose(LifterState state, frc::Translation2d effectorOffset);

 private:
};
