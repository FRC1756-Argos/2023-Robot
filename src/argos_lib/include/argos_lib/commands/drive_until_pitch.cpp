/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "../../argos_lib/include/argos_lib/commands/drive_until_pitch.h"

#include <chrono>

#include "units/angle.h"
#include "units/time.h"

DriveUntilPitch::DriveUntilPitch(SwerveDriveSubsystem* swerveDrive,
                                 units::degree_t velAngle,
                                 double power,
                                 units::degree_t pitchGoal,
                                 units::time::second_t timeout)
    : m_pDrive{swerveDrive}
    , m_velAngle{velAngle}
    , m_power{power}
    , m_startTime{std::chrono::high_resolution_clock::now()}
    , m_pitchGoal{pitchGoal}
    , m_timeout{timeout} {  // * AddRequirements OK here because no child commands are called that depend on subsystem
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void DriveUntilPitch::Initialize() {
  // * Nothing to initialize
}

// Called repeatedly when this Command is scheduled to run
void DriveUntilPitch::Execute() {
  // Cancel if manually overriden
  if (m_pDrive->GetManualOverride()) {
    m_pDrive->SwerveDrive(m_velAngle, m_power);
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
  // * abs because we could be going backwkards up the station too
  if (units::math::abs(m_pDrive->GetRobotPitch()) >= m_pitchGoal) {
    return true;
  } else {
    return false;  ///> Pitch goal was not reached, continue
  }
}
