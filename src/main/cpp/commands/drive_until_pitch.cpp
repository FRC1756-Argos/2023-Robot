/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_until_pitch.h"

#include <chrono>

#include "units/angle.h"
#include "units/time.h"

// REMOVEME debugging
#include <frc/smartdashboard/SmartDashboard.h>
// ! end

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
    , m_velocityRamper{6 / 1_s, 6 / 1_s, initialPower} {
  // * AddRequirements OK here because no child commands are called that depend on subsystem
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void DriveUntilPitch::Initialize() {
  m_velocityRamper.Reset(m_initialPower);
  m_startTime = std::chrono::high_resolution_clock::now();
  // std::printf("Driving to pitch: %0.2f\n", m_pitchGoal.to<double>());
}

// Called repeatedly when this Command is scheduled to run
void DriveUntilPitch::Execute() {
  units::second_t sinceStart{std::chrono::high_resolution_clock::now() - m_startTime};

  // If command goes over alloted time
  if (sinceStart >= m_timeout) {
    m_pDrive->StopDrive();
    Cancel();  // Interrupt the command
    return;
  }

  // Slew rate limiter just increases to 100% power for some reason...
  m_pDrive->SwerveDrive(m_velAngle,
                        std::clamp(m_velocityRamper.Calculate(m_power).to<double>(),
                                   std::min(m_initialPower, m_power),
                                   std::max(m_initialPower, m_power)));
}

// Called once the command ends or is interrupted.
void DriveUntilPitch::End(bool interrupted) {
  /*
  ? If interrupted, present error?
  */
}

// Returns true when the command should end.
bool DriveUntilPitch::IsFinished() {
  bool finished = false;
  auto pitch = m_pDrive->GetRobotPitch();
  // * abs because we could be going backwards up the station too
  switch (m_approachDirection) {
    case ApproachDirection::Increasing:
      finished = pitch >= m_pitchGoal;
      break;
    case ApproachDirection::Decreasing:
      finished = pitch <= m_pitchGoal;
      break;
  }
  // if (finished) {
  //   std::printf("---> pitch: %0.2f\n", pitch.to<double>());
  // }
  // REMOVEME debugging
  frc::SmartDashboard::PutNumber("DriveUntilPitch/Finished? ", static_cast<double>(finished));
  // ! end
  return finished;
}
