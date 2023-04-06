/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/swerve_trapezoidal_spline.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "utils/pose_continuity_fix.h"

SwerveTrapezoidalSpline::SwerveTrapezoidalSpline(
    const frc::Spline<3>::ControlVector initialPosition,
    const units::degree_t initialAngle,
    const std::vector<frc::Translation2d>& waypoints,
    const frc::Spline<3>::ControlVector finalPosition,
    const units::degree_t finalAngle,
    const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints,
    const units::feet_per_second_t initialVelocity,
    const units::feet_per_second_t finalVelocity)
    : m_initialAngle{initialAngle}, m_finalAngle{finalAngle} {
  frc::TrajectoryConfig config{linearConstraints.maxVelocity, linearConstraints.maxAcceleration};
  config.SetStartVelocity(initialVelocity);
  config.SetEndVelocity(finalVelocity);
  m_pathTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(initialPosition, waypoints, finalPosition, config);
}

frc::Trajectory::State SwerveTrapezoidalSpline::Calculate(units::second_t time) const {
  return m_pathTrajectory.Sample(time);
}

bool SwerveTrapezoidalSpline::IsFinished(units::second_t time) const {
  return time >= m_pathTrajectory.TotalTime();
}

units::degree_t SwerveTrapezoidalSpline::GetEndAngle() const {
  return m_finalAngle;
}

units::feet_per_second_t SwerveTrapezoidalSpline::GetXVelocity(const frc::Trajectory::State& state) const {
  return state.velocity * units::math::cos(state.pose.Rotation().Degrees());
}

units::feet_per_second_t SwerveTrapezoidalSpline::GetYVelocity(const frc::Trajectory::State& state) const {
  return state.velocity * units::math::sin(state.pose.Rotation().Degrees());
}
