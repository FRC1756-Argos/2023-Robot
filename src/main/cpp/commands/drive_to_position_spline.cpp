/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_to_position_spline.h"

#include "utils/swerve_trapezoidal_spline.h"

DriveToPositionSpline::DriveToPositionSpline(
    SwerveDriveSubsystem* drive,
    const frc::Spline<3>::ControlVector initialPosition,
    const units::degree_t initialAngle,
    const std::vector<frc::Translation2d>& waypoints,
    const frc::Spline<3>::ControlVector finalPosition,
    const units::degree_t finalAngle,
    const units::second_t turnDelay,
    const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints,
    const frc::TrapezoidProfile<units::degrees>::Constraints rotationalConstraints,
    const units::feet_per_second_t initialVelocity,
    const units::feet_per_second_t finalVelocity)
    : m_pDrive(drive)
    , m_splinePath{initialPosition,
                   initialAngle,
                   waypoints,
                   finalPosition,
                   finalAngle,
                   turnDelay,
                   linearConstraints,
                   initialVelocity,
                   finalVelocity}
    , m_initialAngle{initialAngle}
    , m_finalAngle{finalAngle}
    , m_linearConstraints{linearConstraints}
    , m_rotationalConstraints{rotationalConstraints}
    , m_initialVelocity{initialVelocity}
    , m_finalVelocity{finalVelocity} {
  AddRequirements(drive);

  // Check if triangle profile
  units::degree_t turnDistance = units::math::abs(finalAngle - initialAngle);
  units::second_t turnTime = units::math::sqrt(turnDistance * 4.0 / rotationalConstraints.maxAcceleration);
  // Will turn with trapezoidal profile
  if (turnTime / 2 * rotationalConstraints.maxAcceleration > rotationalConstraints.maxVelocity) {
    turnTime = (turnDistance -
                units::math::pow<2>(rotationalConstraints.maxVelocity) / rotationalConstraints.maxAcceleration) /
               rotationalConstraints.maxVelocity;
  }
  if (turnDelay + turnTime > m_splinePath.GetDriveTime()) {
    m_splinePath.SetTurnDelay(units::math::max(0_s, m_splinePath.GetDriveTime() - turnTime));
  }
}

// Called when the command is initially scheduled.
void DriveToPositionSpline::Initialize() {
  m_pDrive->UpdateFollowerRotationalPIDConstraints(m_rotationalConstraints);
  m_pDrive->StartDrivingProfile(m_splinePath);
}

// Called repeatedly when this Command is scheduled to run
void DriveToPositionSpline::Execute() {
  m_pDrive->SwerveDrive(0, 0, 0);
}

// Called once the command ends or is interrupted.
void DriveToPositionSpline::End(bool interrupted) {
  if (interrupted || units::math::abs(m_finalVelocity) < 0.1_fps) {
    m_pDrive->StopDrive();
  }
}

// Returns true when the command should end.
bool DriveToPositionSpline::IsFinished() {
  return m_pDrive->ProfileIsComplete();
}

frc::Spline<3>::ControlVector ConvertToControlVector(units::foot_t x,
                                                     units::foot_t y,
                                                     units::foot_t tangentX,
                                                     units::foot_t tangentY) {
  return frc::Spline<3>::ControlVector{.x = {units::meter_t{x}.to<double>(), units::meter_t{tangentX}.to<double>()},
                                       .y = {units::meter_t{y}.to<double>(), units::meter_t{tangentY}.to<double>()}};
}
