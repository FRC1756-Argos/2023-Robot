/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/swerve_trapezoidal_profile.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "utils/pose_continuity_fix.h"

SwerveTrapezoidalProfileSegment::SwerveTrapezoidalProfileSegment()
    : SwerveTrapezoidalProfileSegment(
          frc::Pose2d{}, 0_deg, frc::Translation2d{}, 0_deg, frc::TrapezoidProfile<units::inches>::Constraints{}) {}

SwerveTrapezoidalProfileSegment::SwerveTrapezoidalProfileSegment(
    const frc::Pose2d initialPosition,
    const units::degree_t initialAngle,
    const frc::Translation2d relativeTranslation,
    const units::degree_t relativeRotation,
    const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints)
    : m_initialPosition{initialPosition}
    , m_initialAngle{initialAngle}
    , m_relativeTranslation{relativeTranslation}
    , m_relativeRotation{relativeRotation}
    , m_linearProfile{linearConstraints, frc::TrapezoidProfile<units::inches>::State{relativeTranslation.Norm(), 0_mps}}
    , m_motionAngle{units::math::atan2(m_relativeTranslation.Y(), m_relativeTranslation.X())} {
  frc::Pose2d finalPose{m_initialPosition.Translation() + m_relativeTranslation,
                        m_initialPosition.Rotation() + frc::Rotation2d{m_relativeRotation}};
  m_odometryOffsetAngle =
      GetContinuousOffset(m_initialAngle, m_initialPosition, m_initialAngle + m_relativeRotation, finalPose);
  frc::SmartDashboard::PutNumber("(SwerveFollower) Relative X", units::inch_t{m_relativeTranslation.X()}.to<double>());
  frc::SmartDashboard::PutNumber("(SwerveFollower) Relative Y", units::inch_t{m_relativeTranslation.Y()}.to<double>());
}

SwerveTrapezoidalProfileSegment::SwerveTrapezoidalProfileSegment(
    const frc::Pose2d initialPosition,
    const units::degree_t initialAngle,
    const frc::Pose2d finalPosition,
    const units::degree_t finalAngle,
    const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints)
    : SwerveTrapezoidalProfileSegment(initialPosition,
                                      initialAngle,
                                      finalPosition.Translation() - initialPosition.Translation(),
                                      finalAngle - initialAngle,
                                      linearConstraints) {
  frc::SmartDashboard::PutNumber("(TrapezoidalProfile) Initial Position X",
                                 units::inch_t{initialPosition.X()}.to<double>());
  frc::SmartDashboard::PutNumber("(TrapezoidalProfile) Initial Position Y",
                                 units::inch_t{initialPosition.Y()}.to<double>());
  frc::SmartDashboard::PutNumber("(TrapezoidalProfile) Initial Position Angle",
                                 initialPosition.Rotation().Degrees().to<double>());
  frc::SmartDashboard::PutNumber("(TrapezoidalProfile) Final Position X",
                                 units::inch_t{finalPosition.X()}.to<double>());
  frc::SmartDashboard::PutNumber("(TrapezoidalProfile) Final Position Y",
                                 units::inch_t{finalPosition.Y()}.to<double>());
  frc::SmartDashboard::PutNumber("(TrapezoidalProfile) Final Position Angle",
                                 finalPosition.Rotation().Degrees().to<double>());
  auto translation = finalPosition.Translation() - initialPosition.Translation();
  frc::SmartDashboard::PutNumber("(TrapezoidalProfile) Translation X", units::inch_t{translation.X()}.to<double>());
  frc::SmartDashboard::PutNumber("(TrapezoidalProfile) Translation Y", units::inch_t{translation.Y()}.to<double>());
}

frc::Trajectory::State SwerveTrapezoidalProfileSegment::Calculate(units::second_t time) const {
  const auto linearState = m_linearProfile.Calculate(time);
  const auto newTranslation =
      m_relativeTranslation * (linearState.position / m_relativeTranslation.Norm()).to<double>();
  frc::SmartDashboard::PutNumber("(SwerveFollower) Linear Position", linearState.position.to<double>());
  frc::SmartDashboard::PutNumber("(SwerveFollower) Linear Length",
                                 units::inch_t{m_relativeTranslation.Norm()}.to<double>());
  frc::SmartDashboard::PutNumber("(SwerveFollower) Completion %",
                                 (linearState.position / m_relativeTranslation.Norm()).to<double>());
  // const auto newRotation = m_relativeRotation * (rotationalState.position / m_relativeRotation.Degrees()).to<double>();
  return frc::Trajectory::State{time,
                                linearState.velocity,
                                0_mps_sq,
                                frc::Pose2d{m_initialPosition.Translation() + newTranslation, m_motionAngle},
                                units::curvature_t{0}};
}

bool SwerveTrapezoidalProfileSegment::IsFinished(units::second_t time) const {
  return m_linearProfile.IsFinished(time);
}

units::degree_t SwerveTrapezoidalProfileSegment::GetEndAngle() const {
  return m_initialAngle + m_relativeRotation;
}

units::degree_t SwerveTrapezoidalProfileSegment::GetOdometryOffset() const {
  return m_odometryOffsetAngle;
}

units::feet_per_second_t SwerveTrapezoidalProfileSegment::GetXVelocity(const frc::Trajectory::State& state) const {
  return state.velocity * units::math::cos(m_motionAngle);
}

units::feet_per_second_t SwerveTrapezoidalProfileSegment::GetYVelocity(const frc::Trajectory::State& state) const {
  return state.velocity * units::math::sin(m_motionAngle);
}

SwerveTrapezoidalProfile::SwerveTrapezoidalProfile() = default;
