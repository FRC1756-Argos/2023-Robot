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

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutonomousLeftSlamGrabBalance
    : public frc2::CommandHelper<frc2::CommandBase, AutonomousLeftSlamGrabBalance>
    , public AutonomousCommand {
 public:
  AutonomousLeftSlamGrabBalance(SwerveDriveSubsystem& drive,
                                SimpleLedSubsystem& leds,
                                OuiOuiPlacerSubsystem& ouiOui,
                                LifterSubsystem& lifter,
                                BashGuardSubsystem& bash,
                                IntakeSubsystem& intake);

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
  SimpleLedSubsystem& m_leds;
  OuiOuiPlacerSubsystem& m_ouiOui;
  LifterSubsystem& m_lifter;
  BashGuardSubsystem& m_bash;
  IntakeSubsystem& m_intake;

  frc2::CommandPtr m_allCommands;
};
