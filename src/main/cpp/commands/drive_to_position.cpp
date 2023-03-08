/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_to_position.h"

#include "utils/swerve_trapezoidal_profile.h"

DriveToPosition::DriveToPosition(SwerveDriveSubsystem* drive,
                                 const frc::Pose2d source,
                                 const units::degree_t sourceAngle,
                                 const frc::Pose2d destination,
                                 const units::degree_t destAngle,
                                 const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints,
                                 const frc::TrapezoidProfile<units::degrees>::Constraints rotationalConstraints,
                                 const units::feet_per_second_t initialVelocity,
                                 const units::feet_per_second_t finalVelocity)
    : m_pDrive(drive)
    , m_source(source)
    , m_sourceAngle(sourceAngle)
    , m_destination(destination)
    , m_destAngle{destAngle}
    , m_linearConstraints(linearConstraints)
    , m_rotationalConstraints(rotationalConstraints)
    , m_initialVelocity{initialVelocity}
    , m_finalVelocity{finalVelocity} {
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void DriveToPosition::Initialize() {
  m_pDrive->UpdateFollowerRotationalPIDConstraints(m_rotationalConstraints);
  m_pDrive->StartDrivingProfile(SwerveTrapezoidalProfileSegment{
      m_source, m_sourceAngle, m_destination, m_destAngle, m_linearConstraints, m_initialVelocity, m_finalVelocity});
}

// Called repeatedly when this Command is scheduled to run
void DriveToPosition::Execute() {
  m_pDrive->SwerveDrive(0, 0, 0);
}

// Called once the command ends or is interrupted.
void DriveToPosition::End(bool interrupted) {
  if (interrupted || units::math::abs(m_finalVelocity) < 0.1_fps) {
    m_pDrive->StopDrive();
  }
}

// Returns true when the command should end.
bool DriveToPosition::IsFinished() {
  return m_pDrive->ProfileIsComplete();
}
