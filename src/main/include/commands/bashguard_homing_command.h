/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <argos_lib/general/debouncer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/bash_guard_subsystem.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class BashguardHomingCommand : public frc2::CommandHelper<frc2::CommandBase, BashguardHomingCommand> {
 public:
  explicit BashguardHomingCommand(BashGuardSubsystem& subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  BashGuardSubsystem& m_bashGuardSubsytem;
  argos_lib::Debouncer m_bashGuardMovingDebounce;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
};
