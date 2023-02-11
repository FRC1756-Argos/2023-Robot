/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/lifter_kinematics.h"

#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/math.h>

#include <cmath>

namespace {
  auto Squared(auto val) {
    return units::math::pow<2, decltype(val)>(val);
  }
}  // namespace

LifterKinematics::LifterKinematics(const frc::Translation2d& fulcrumPosition,
                                   const units::meter_t armRotationOffset,
                                   const frc::Translation2d& effectorOffset)
    : m_fulcrumPosition(fulcrumPosition), m_armRotationOffset(armRotationOffset), m_effectorOffset(effectorOffset) {}

// effectorYOffset is the effector pose offset from rotation center
LifterState LifterKinematics::GetJoints(frc::Translation2d pose, bool effectorInverted) const {
  const auto endEffectorOffset =
      effectorInverted ? frc::Translation2d{m_effectorOffset.X(), -m_effectorOffset.Y()} : m_effectorOffset;
  units::meter_t poseMagnitude = pose.Distance(m_fulcrumPosition);
  units::meter_t sideB = units::math::sqrt(Squared(poseMagnitude - endEffectorOffset.X()) -
                                           Squared(endEffectorOffset.Y() + m_armRotationOffset));
  units::meter_t sideA = m_armRotationOffset + endEffectorOffset.Y();
  units::angle::radian_t alpha = units::math::atan2(sideA, sideB + endEffectorOffset.X());
  units::angle::radian_t phi = units::math::atan2(pose.Y() - m_fulcrumPosition.Y(), pose.X() - m_fulcrumPosition.X());
  units::angle::radian_t theta = phi - alpha;

  return LifterState{sideB, theta};
}

frc::Translation2d LifterKinematics::GetPose(LifterState state, bool effectorInverted) const {
  units::radian_t rotation = state.shoulderAngle;

  frc::Translation2d initPosition{state.armLen, m_effectorOffset.Y()};

  frc::Translation2d solvedPosition = frc::Translation2d(initPosition.RotateBy(rotation));

  return solvedPosition;
}
