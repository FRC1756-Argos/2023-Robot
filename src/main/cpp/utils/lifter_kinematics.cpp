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

LifterKinematics::LifterKinematics() = default;

LifterState LifterKinematics::GetJoints(frc::Translation2d pose, frc::Transform2d effectorOffset) {
  units::meter_t poseMagnitude = pose.Distance(frc::Translation2d(0_m, 0_m));

  auto arm_len = units::math::sqrt(units::math::pow<2, units::meter_t>(poseMagnitude) -
                                   units::math::pow<2, units::meter_t>(units::math::abs(effectorOffset.Y())));
  auto shoulderAngle =
      units::math::atan2<units::length::meter_t, units::length::meter_t>(pose.Y(), pose.X()) -
      units::math::atan(units::math::sqrt(units::math::pow<2, units::meter_t>(poseMagnitude) -
                                          units::math::pow<2, units::meter_t>(units::math::abs(effectorOffset.Y()))) /
                        units::math::abs(effectorOffset.Y()));

  LifterState lift_state{arm_len, shoulderAngle};

  return lift_state;
}

frc::Translation2d LifterKinematics::GetPose(LifterState state, frc::Transform2d effectorOffset) {
  units::radian_t rotation =
      (units::make_unit<units::radian_t>(std::numbers::pi_v<double> * 2) -
       units::math::atan2<units::length::meter_t, units::length::meter_t>(effectorOffset.Y(), state.armLen)) +
      state.shoulderAngle + units::math::atan(state.armLen / units::math::abs(effectorOffset.Y()));

  frc::Translation2d initPosition{effectorOffset.X(), effectorOffset.Y()};

  frc::Translation2d solvedPosition = initPosition.RotateBy(rotation);

  return solvedPosition;
}
