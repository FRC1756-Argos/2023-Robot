/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <commands/set_arm_pose_command.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <subsystems/bash_guard_subsystem.h>
#include <subsystems/intake_subsystem.h>
#include <subsystems/lifter_subsystem.h>

#include <memory>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class GripConeCommand : public frc2::CommandHelper<frc2::CommandBase, GripConeCommand> {
 public:
  GripConeCommand(LifterSubsystem& lifter, BashGuardSubsystem& bash, IntakeSubsystem& intake);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  LifterSubsystem& m_lifter;
  IntakeSubsystem& m_intake;
  BashGuardSubsystem& m_bash;

  frc2::CommandPtr m_allCommands;
};
