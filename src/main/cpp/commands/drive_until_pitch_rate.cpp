/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_until_pitch_rate.h"

DriveUntilPitchRate::DriveUntilPitchRate(SwerveDriveSubsystem* swerveDrive,
                                         units::degree_t velAngle,
                                         double power,
                                         double initialPower,
                                         units::degrees_per_second_t pitchRateGoal,
                                         ApproachDirection approachDirection,
                                         units::time::second_t timeout)
    : m_pDrive{swerveDrive}
    , m_velAngle{velAngle}
    , m_power{power}
    , m_initialPower{initialPower}
    , m_startTime{std::chrono::high_resolution_clock::now()}
    , m_pitchRateGoal{pitchRateGoal}
    , m_approachDirection{approachDirection}
    , m_timeout{timeout}
    , m_velocityRamper{6 / 1_s, 6 / 1_s, initialPower} {
  // * AddRequirements OK here because no child commands are called that depend on subsystem
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void DriveUntilPitchRate::Initialize() {
  m_velocityRamper.Reset(m_initialPower);
  m_startTime = std::chrono::high_resolution_clock::now();
  // std::printf("Driving to pitch rate: %0.2f\n", m_pitchRateGoal.to<double>());
}

// Called repeatedly when this Command is scheduled to run
void DriveUntilPitchRate::Execute() {
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
void DriveUntilPitchRate::End(bool interrupted) {
  /*
  ? If interrupted, present error?
  */
}

// Returns true when the command should end.
bool DriveUntilPitchRate::IsFinished() {
  // * abs because we could be going backwards up the station too
  bool finished = false;
  auto pitchRate = m_pDrive->GetRobotPitchRate();
  switch (m_approachDirection) {
    case ApproachDirection::Increasing:
      finished = pitchRate >= m_pitchRateGoal;
      break;
    case ApproachDirection::Decreasing:
      finished = pitchRate <= m_pitchRateGoal;
      break;
  }
  // if (finished) {
  //   std::printf("---> pitch rate: %0.2f\n", pitchRate.to<double>());
  // }
  return finished;
}
