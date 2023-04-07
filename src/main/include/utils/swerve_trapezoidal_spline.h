/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/spline/Spline.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <vector>

class SwerveTrapezoidalSpline {
 public:
  SwerveTrapezoidalSpline() = delete;
  SwerveTrapezoidalSpline(const frc::Spline<3>::ControlVector initialPosition,
                          const units::degree_t initialAngle,
                          const std::vector<frc::Translation2d>& waypoints,
                          const frc::Spline<3>::ControlVector finalPosition,
                          const units::degree_t finalAngle,
                          const units::second_t turnDelay,
                          const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints,
                          const units::feet_per_second_t initialVelocity = 0_fps,
                          const units::feet_per_second_t finalVelocity = 0_fps);
  SwerveTrapezoidalSpline(const SwerveTrapezoidalSpline& other) = default;
  SwerveTrapezoidalSpline(SwerveTrapezoidalSpline&& other) = default;

  SwerveTrapezoidalSpline& operator=(const SwerveTrapezoidalSpline&) = delete;
  SwerveTrapezoidalSpline& operator=(SwerveTrapezoidalSpline&&) = delete;

  frc::Trajectory::State Calculate(units::second_t time) const;
  units::degree_t HeadingSetpoint(units::second_t time) const;
  bool IsFinished(units::second_t time) const;
  units::degree_t GetEndAngle() const;
  units::degree_t GetOdometryOffset() const;

  units::second_t GetDriveTime() const;
  void SetTurnDelay(units::second_t delay);

  units::feet_per_second_t GetXVelocity(const frc::Trajectory::State& state) const;
  units::feet_per_second_t GetYVelocity(const frc::Trajectory::State& state) const;

 private:
  frc::Trajectory m_pathTrajectory;
  units::degree_t m_initialAngle;
  units::degree_t m_finalAngle;
  units::second_t m_turnDelay;
};
