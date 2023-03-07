/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <commands/autonomous/autonomous_command.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <string>

#include "subsystems/bash_guard_subsystem.h"
#include "subsystems/intake_subsystem.h"
#include "subsystems/lifter_subsystem.h"
#include "subsystems/simple_led_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutonomousLoadingStationConeCubeScore
    : public frc2::CommandHelper<frc2::CommandBase, AutonomousLoadingStationConeCubeScore>
    , public AutonomousCommand {
 public:
  AutonomousLoadingStationConeCubeScore(SwerveDriveSubsystem& drive,
                                        BashGuardSubsystem& bash,
                                        LifterSubsystem& lifter,
                                        IntakeSubsystem& intake,
                                        SimpleLedSubsystem& leds);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  /**
   * @copydoc AutonomousCommand::GetName()
   */
  std::string GetName() const final;
  /**
   * @copydoc AutonomousCommand::GetCommand()
   */
  frc2::Command* GetCommand() final;

 private:
  SwerveDriveSubsystem& m_drive;
  BashGuardSubsystem& m_bashGuard;
  LifterSubsystem& m_lifter;
  IntakeSubsystem& m_intake;
  SimpleLedSubsystem& m_leds;

  frc2::CommandPtr m_allCommands;
};
