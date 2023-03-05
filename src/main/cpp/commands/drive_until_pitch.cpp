/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_until_pitch.h"

#include <chrono>

#include "units/angle.h"
#include "units/time.h"

DriveUntilPitch::DriveUntilPitch(SwerveDriveSubsystem* swerveDrive,
                                 units::degree_t velAngle,
                                 double power,
                                 double initialPower,
                                 units::degree_t pitchGoal,
                                 ApproachDirection approachDirection,
                                 units::time::second_t timeout)
    : m_pDrive{swerveDrive}
    , m_velAngle{velAngle}
    , m_power{power}
    , m_initialPower{initialPower}
    , m_startTime{std::chrono::high_resolution_clock::now()}
    , m_pitchGoal{pitchGoal}
    , m_approachDirection{approachDirection}
    , m_timeout{timeout}
    , m_velocityRamper{2 / 1_s, 2 / 1_s, initialPower} {
  // * AddRequirements OK here because no child commands are called that depend on subsystem
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void DriveUntilPitch::Initialize() {
  m_velocityRamper.Reset(m_initialPower);
}

// Called repeatedly when this Command is scheduled to run
void DriveUntilPitch::Execute() {
  // Cancel if manually overriden
  if (m_pDrive->GetManualOverride()) {
    m_pDrive->SwerveDrive(m_velAngle, m_velocityRamper.Calculate(m_power).to<double>());
  } else {
    Cancel();
  }

  std::chrono::seconds sinceStart =
      std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - m_startTime);

  // If command goes over alloted time
  if (sinceStart.count() >= m_timeout.to<double>()) {
    Cancel();  // Interrupt the command
  }
}

// Called once the command ends or is interrupted.
void DriveUntilPitch::End(bool interrupted) {
  /*
  ? If interrupted, present error?
  */
}

// Returns true when the command should end.
bool DriveUntilPitch::IsFinished() {
  // * abs because we could be going backwards up the station too
  switch (m_approachDirection) {
    case ApproachDirection::Increasing:
      return m_pDrive->GetRobotPitch() >= m_pitchGoal;
    case ApproachDirection::Decreasing:
      return m_pDrive->GetRobotPitch() <= m_pitchGoal;
  }
  return true;
}
