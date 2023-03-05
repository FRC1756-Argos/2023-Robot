/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include <chrono>

#include "Constants.h"
#include "subsystems/swerve_drive_subsystem.h"

class DriveUntilPitchRate : public frc2::CommandHelper<frc2::CommandBase, DriveUntilPitchRate> {
 public:
  DriveUntilPitchRate(SwerveDriveSubsystem* swerveDrive,
                      units::degree_t velAngle,
                      double power,
                      double initialPower,
                      units::degrees_per_second_t pitchRateGoal,
                      ApproachDirection approachDirection,
                      units::time::second_t timeout);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem* m_pDrive;    ///< Raw pointer to swerve drive subsystem object
  const units::degree_t m_velAngle;  ///< Angle of drive direction relative to field-centric
  const double m_power;              ///< Power to apply in m_velAngle direction as percent output ([0, 1])
  const double m_initialPower;       ///< Power prior to command start [-1,1]
  std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;  ///< Start time as a time point
  const units::degrees_per_second_t m_pitchRateGoal;  ///< The pitch rate the robot has to be at to complete the command
  const ApproachDirection m_approachDirection;        ///< Threshold approach direction for pitch goal
  const units::time::second_t m_timeout;  ///< The amount of time allowed to pass before the command times out
  frc::SlewRateLimiter<units::scalar> m_velocityRamper;  ///< Limit acceleration
};
