/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/bashguard_homing_command.h"

#include <chrono>

using namespace std::chrono_literals;

BashguardHomingCommand::BashguardHomingCommand(BashGuardSubsystem& subsystem)
    : m_bashGuardSubsytem(subsystem), m_bashGuardMovingDebounce{{0_ms, 250_ms}, true} {}

// Called when the command is initially scheduled.
void BashguardHomingCommand::Initialize() {
  m_bashGuardSubsytem.SetExtensionSpeed(-0.1);
  m_bashGuardSubsytem.SetBashGuardManualOverride(false);
  m_startTime = std::chrono::steady_clock::now();
  m_bashGuardMovingDebounce.Reset(true);
}

// Called repeatedly when this Command is scheduled to run
void BashguardHomingCommand::Execute() {
  if (m_bashGuardSubsytem.IsBashGuardManualOverride() || (std::chrono::steady_clock::now() - m_startTime) > 4.0s) {
    Cancel();
  } else {
    m_bashGuardSubsytem.SetExtensionSpeed(-0.1);
  }
}

// Called once the command ends or is interrupted.
void BashguardHomingCommand::End(bool interrupted) {
  if (!interrupted) {
    m_bashGuardSubsytem.UpdateExtensionHome();
  }
  m_bashGuardSubsytem.SetExtensionSpeed(0.0);
}

// Returns true when the command should end.
bool BashguardHomingCommand::IsFinished() {
  return !m_bashGuardMovingDebounce(m_bashGuardSubsytem.IsBashGuardMoving());
}
