/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/lifter_kinematics.h"

#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>

#include <cmath>
#include <iostream>
#include <numbers>

#include "units/math.h"

namespace {
  auto Squared(auto val) {
    return units::math::pow<2, decltype(val)>(val);
  }
}  // namespace

LifterKinematics::LifterKinematics() = default;

// effectorYOffset is the effector pose offset from rotation center
LifterState LifterKinematics::GetJoints(frc::Translation2d pose, units::meter_t effectorYOffset) {
  units::meter_t poseMagnitude = pose.Distance(frc::Translation2d(0_m, 0_m));
  units::meter_t sideB = units::math::sqrt(Squared(poseMagnitude) + Squared(effectorYOffset));
  units::meter_t sideA = units::math::abs(effectorYOffset);
  units::angle::radian_t alpha = units::math::atan(sideB / sideA);
  units::angle::radian_t phi = units::math::atan2(pose.Y(), pose.X());
  units::angle::radian_t theta = phi - alpha;

  return LifterState{sideB, theta};
}

frc::Translation2d LifterKinematics::GetPose(LifterState state, frc::Translation2d effectorOffset) {
  units::radian_t rotation = state.shoulderAngle;

  frc::Translation2d initPosition{state.armLen, effectorOffset.Y()};

  frc::Translation2d solvedPosition = initPosition.RotateBy(rotation);

  return solvedPosition;
}
