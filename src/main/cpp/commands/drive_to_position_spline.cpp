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
