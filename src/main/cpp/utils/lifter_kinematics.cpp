/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/lifter_kinematics.h"

#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/base.h>
#include <units/dimensionless.h>
#include <units/math.h>

#include <cmath>

namespace {
  auto Squared(auto val) {
    return units::math::pow<2, decltype(val)>(val);
  }
}  // namespace

LifterKinematics::LifterKinematics(const frc::Translation2d& fulcrumPosition,
                                   const units::meter_t armRotationOffset,
                                   const frc::Translation2d& effectorOffset,
                                   const frc::Translation2d& fixedBoomActuatorPosition,
                                   const frc::Translation2d& actuatedBoomActuatorPosition)
    : m_fulcrumPosition(fulcrumPosition)
    , m_armRotationOffset(armRotationOffset)
    , m_effectorOffset(effectorOffset)
    , m_fixedBoomAnchorToFulcrumDist(units::math::hypot(fulcrumPosition.X() - fixedBoomActuatorPosition.X(),
                                                        fulcrumPosition.Y() - fixedBoomActuatorPosition.Y()))
    , m_fixedBoomAnchorToFulcrumAngle(units::math::atan2(fulcrumPosition.X() - fixedBoomActuatorPosition.X(),
                                                         fulcrumPosition.Y() - fixedBoomActuatorPosition.Y()))
    , m_articulatedBoomAnchorToFulcrumDist(
          units::math::hypot(actuatedBoomActuatorPosition.X(), actuatedBoomActuatorPosition.Y()))
    , m_articulatedBoomAnchorToFulcrumAngle(
          units::math::atan2(actuatedBoomActuatorPosition.Y(), actuatedBoomActuatorPosition.X())) {}

// effectorYOffset is the effector pose offset from rotation center
ArmState LifterKinematics::GetJointsFromEffector(const frc::Translation2d& pose, bool effectorInverted) const {
  const auto endEffectorOffset =
      effectorInverted ? frc::Translation2d{m_effectorOffset.X(), -m_effectorOffset.Y()} : m_effectorOffset;
  return GetJoints(pose, endEffectorOffset);
}

ArmState LifterKinematics::GetJoints(const frc::Translation2d& pose, const frc::Translation2d& effectorOffset) const {
  units::meter_t poseMagnitude = pose.Distance(m_fulcrumPosition);
  units::meter_t sideB = units::math::sqrt(Squared(poseMagnitude - effectorOffset.X()) -
                                           Squared(effectorOffset.Y() + m_armRotationOffset));
  units::meter_t sideA = m_armRotationOffset + effectorOffset.Y();
  units::angle::radian_t alpha = units::math::atan2(sideA, sideB + effectorOffset.X());
  units::angle::radian_t phi = units::math::atan2(pose.Y() - m_fulcrumPosition.Y(), pose.X() - m_fulcrumPosition.X());
  units::angle::radian_t theta = phi - alpha;

  return ArmState{sideB, theta};
}

ArmState LifterKinematics::GetJoints(const frc::Translation2d& pose) const {
  return GetJoints(pose, {0_in, 0_in});
}

frc::Translation2d LifterKinematics::GetEffectorPose(const ArmState& state, bool effectorInverted) const {
  const auto endEffectorOffset =
      effectorInverted ? frc::Translation2d{m_effectorOffset.X(), -m_effectorOffset.Y()} : m_effectorOffset;

  return GetPose(state, endEffectorOffset);
}

frc::Translation2d LifterKinematics::GetPose(const ArmState& state) const {
  return GetPose(state, {0_in, 0_in});
}

frc::Translation2d LifterKinematics::GetPose(const ArmState& state, const frc::Translation2d& effectorOffset) const {
  units::radian_t rotation = state.shoulderAngle;

  frc::Translation2d initPosition{state.armLen + effectorOffset.X(), effectorOffset.Y() + m_armRotationOffset};

  frc::Translation2d solvedPosition = frc::Translation2d(initPosition.RotateBy(rotation));

  solvedPosition = solvedPosition + m_fulcrumPosition;

  return solvedPosition;
}

units::inch_t LifterKinematics::ShoulderAngleToBoomExtension(units::degree_t shoulderAngle) const {
  return units::math::sqrt(Squared(m_fixedBoomAnchorToFulcrumDist) + Squared(m_articulatedBoomAnchorToFulcrumDist) -
                           2 * m_fixedBoomAnchorToFulcrumDist * m_articulatedBoomAnchorToFulcrumDist *
                               units::math::cos(90_deg + m_articulatedBoomAnchorToFulcrumAngle +
                                                m_fixedBoomAnchorToFulcrumAngle + shoulderAngle));
}

units::degree_t LifterKinematics::BoomExtensionToShoulderAngle(units::inch_t boomExtension) const {
  return units::math::acos((Squared(m_fixedBoomAnchorToFulcrumDist) + Squared(m_articulatedBoomAnchorToFulcrumDist) -
                            Squared(boomExtension)) /
                           (2 * m_fixedBoomAnchorToFulcrumDist * m_articulatedBoomAnchorToFulcrumDist)) -
         90_deg - m_articulatedBoomAnchorToFulcrumAngle - m_fixedBoomAnchorToFulcrumAngle;
}

units::velocity::inches_per_second_t LifterKinematics::ShoulderVelocityToBoomVelocity(
    units::radians_per_second_t shoulderVelocity, units::degree_t shoulderAngle) const {
  // This mess is the derivative wrt time of ShoulderAngleToBoomExtension.
  // Shoulder angle is the only parameter that changes with time
  // Math cheat: https://www.wolframalpha.com/input?i=d%2Fdt+%73qrt(a^2%2Bb^2-2*a*b*co%73(d%2Bx(t)))
  // Note units library doesn't like erasing unitless radian, so that's why we divide velocity by 1 radian
  return (m_fixedBoomAnchorToFulcrumDist * m_articulatedBoomAnchorToFulcrumDist * shoulderVelocity / 1_rad *
          units::math::sin(90_deg + m_articulatedBoomAnchorToFulcrumAngle + m_fixedBoomAnchorToFulcrumAngle +
                           shoulderAngle)) /
         units::math::sqrt(Squared(m_fixedBoomAnchorToFulcrumDist) + Squared(m_articulatedBoomAnchorToFulcrumDist) -
                           2 * m_fixedBoomAnchorToFulcrumDist * m_articulatedBoomAnchorToFulcrumDist *
                               units::math::cos(90_deg + m_articulatedBoomAnchorToFulcrumAngle +
                                                m_fixedBoomAnchorToFulcrumAngle + shoulderAngle));
}

units::radians_per_second_t LifterKinematics::BoomVelocityToShoulderVelocity(
    units::velocity::inches_per_second_t boomVelocity, units::inch_t boomPosition) const {
  // This mess is the derivative wrt time of BoomExtensionToShoulderAngle.
  // boom position is the only parameter that changes with time
  // Math cheat: https://www.wolframalpha.com/input?i=d%2Fdt+aco%73((a^2%2Bb^2-y(t)^2)%2F(2ab))-d
  // Note units library doesn't like adding unitless radian, so that's why we multiply by 1 radian
  return 1_rad * (boomPosition * boomVelocity) /
         (m_fixedBoomAnchorToFulcrumDist * m_articulatedBoomAnchorToFulcrumDist *
          units::math::sqrt(
              1 - Squared(Squared(m_fixedBoomAnchorToFulcrumDist) + Squared(m_articulatedBoomAnchorToFulcrumDist) -
                          Squared(boomPosition)) /
                      (4 * Squared(m_fixedBoomAnchorToFulcrumDist) * Squared(m_articulatedBoomAnchorToFulcrumDist))));
}
