/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_to_position.h"

#include "utils/swerve_trapezoidal_profile.h"

DriveToPosition::DriveToPosition(std::shared_ptr<SwerveDriveSubsystem> drive,
                                 const frc::Pose2d initPos,
                                 const units::degree_t initAngle,
                                 const frc::Pose2d dest,
                                 const units::degree_t destAngle,
                                 const frc::TrapezoidProfile<units::inches>::Constraints linConstraints,
                                 frc::TrapezoidProfile<units::degrees>::Constraints rotConstraints)
    : m_drive{drive}
    , m_initPosition{initPos}
    , m_initAngle{initAngle}
    , m_destPosition{dest}
    , m_destAngle{destAngle}
    , m_linearConstraints{linConstraints}
    , m_rotationalConstraints{rotConstraints} {
  // Use addRequirements() here to declare subsystem dependencies.
  // TODO ask david if we want to do add requirements
}

// Called when the command is initially scheduled.
void DriveToPosition::Initialize() {
  // Update rotational PID to reflect constraints
  m_drive->UpdateFollowerRotationalPIDConstraints(m_rotationalConstraints);
  m_drive->StartDrivingProfile(
      SwerveTrapezoidalProfileSegment(m_initPosition, m_initAngle, m_destPosition, m_destAngle, m_linearConstraints));
}

// Called repeatedly when this Command is scheduled to run
void DriveToPosition::Execute() {}

// Called once the command ends or is interrupted.
void DriveToPosition::End(bool interrupted) {
  m_drive->StopDrive();
}

// Returns true when the command should end.
bool DriveToPosition::IsFinished() {
  return m_drive->ProfileIsComplete();
}
