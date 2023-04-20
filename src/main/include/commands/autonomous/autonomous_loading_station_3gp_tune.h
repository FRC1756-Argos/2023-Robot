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
#include "subsystems/oui_oui_placer_subsystem.h"
#include "subsystems/simple_led_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"

class AutonomousLoadingStation3GPTune
    : public frc2::CommandHelper<frc2::CommandBase, AutonomousLoadingStation3GPTune>
    , public AutonomousCommand {
 public:
  AutonomousLoadingStation3GPTune(SwerveDriveSubsystem& drive,
                                  BashGuardSubsystem& bash,
                                  LifterSubsystem& lifter,
                                  IntakeSubsystem& intake,
                                  SimpleLedSubsystem& leds,
                                  OuiOuiPlacerSubsystem& placer);

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
  OuiOuiPlacerSubsystem& m_placer;

  frc2::CommandPtr m_allCommands;
};
