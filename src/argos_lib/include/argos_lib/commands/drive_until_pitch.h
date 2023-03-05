/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <chrono>

#include "Constants.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "units/time.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveUntilPitch : public frc2::CommandHelper<frc2::CommandBase, DriveUntilPitch> {
 public:
  DriveUntilPitch(SwerveDriveSubsystem* swerveDrive,
                  units::degree_t velAngle,
                  double power,
                  units::degree_t pitchGoal,
                  units::time::second_t timeout);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem* m_pDrive;    ///> Raw pointer to swerve drive subsystem object
  const units::degree_t m_velAngle;  ///> Angle of drive direction relative to field-centric
  const double m_power;              ///> Power to apply in m_velAngle direction as percent output ([0, 1])
  const std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;  ///> Start time as a time point
  const units::degree_t m_pitchGoal;      ///> The pitch the robot has to be at to complete the command
  const units::time::second_t m_timeout;  ///> The amount of time allowed to pass before the command times out
};
