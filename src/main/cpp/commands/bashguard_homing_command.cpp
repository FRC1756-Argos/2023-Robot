/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/bashguard_homing_command.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <chrono>

using namespace std::chrono_literals;

BashGuardHomingCommand::BashGuardHomingCommand(BashGuardSubsystem& subsystem)
    : m_bashGuardSubsytem(subsystem), m_bashGuardMovingDebounce{{0_ms, 250_ms}, true} {}

// Called when the command is initially scheduled.
void BashGuardHomingCommand::Initialize() {
  m_bashGuardSubsytem.SetExtensionSpeed(-0.1);
  m_bashGuardSubsytem.SetBashGuardManualOverride(false);
  m_startTime = std::chrono::steady_clock::now();
  m_bashGuardMovingDebounce.Reset(true);
}

// Called repeatedly when this Command is scheduled to run
void BashGuardHomingCommand::Execute() {
  auto timePassed = units::second_t{std::chrono::steady_clock::now() - m_startTime};

  if (m_bashGuardSubsytem.IsBashGuardManualOverride() || (timePassed) > 1.5_s) {
    Cancel();
  } else {
    m_bashGuardSubsytem.SetExtensionSpeed(-0.1);
  }
}

// Called once the command ends or is interrupted.
void BashGuardHomingCommand::End(bool interrupted) {
  if (!interrupted) {
    m_bashGuardSubsytem.UpdateBashGuardHome();
    m_bashGuardSubsytem.SetHomeFailed(false);
  } else {
    m_bashGuardSubsytem.SetExtensionSpeed(0.0);
    m_bashGuardSubsytem.SetHomeFailed(true);
  }
}

// Returns true when the command should end.
bool BashGuardHomingCommand::IsFinished() {
  return !m_bashGuardMovingDebounce(m_bashGuardSubsytem.IsBashGuardMoving());
}
