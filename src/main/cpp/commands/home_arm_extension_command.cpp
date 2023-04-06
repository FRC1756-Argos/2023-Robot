/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/home_arm_extension_command.h"

#include <chrono>

using namespace std::chrono_literals;

HomeArmExtensionCommand::HomeArmExtensionCommand(LifterSubsystem& subsystem)
    : m_lifterSubsystem(subsystem), m_armMovingDebounce{{0_ms, 250_ms}, true} {}

// Called when the command is initially scheduled.
void HomeArmExtensionCommand::Initialize() {
  m_lifterSubsystem.SetArmExtensionSpeed(-0.1);
  m_lifterSubsystem.SetExtensionManualOverride(false);
  m_startTime = std::chrono::steady_clock::now();
  m_armMovingDebounce.Reset(true);
  // Disable soft limits while trying to home
  m_lifterSubsystem.DisableArmExtensionSoftLimits();
}

// Called repeatedly when this Command is scheduled to run
void HomeArmExtensionCommand::Execute() {
  if (m_lifterSubsystem.IsExtensionManualOverride() || (std::chrono::steady_clock::now() - m_startTime) > 4.0s) {
    Cancel();
  } else {
    m_lifterSubsystem.SetArmExtensionSpeed(-0.1);
  }
}

// Called once the command ends or is interrupted.
void HomeArmExtensionCommand::End(bool interrupted) {
  if (!interrupted) {
    m_lifterSubsystem.UpdateArmExtensionHome();
    // Re-enable soft limits
    m_lifterSubsystem.EnableArmExtensionSoftLimits();
  }
  m_lifterSubsystem.SetArmExtensionSpeed(0.0);
}

// Returns true when the command should end.
bool HomeArmExtensionCommand::IsFinished() {
  return !m_armMovingDebounce(m_lifterSubsystem.IsArmExtensionMoving());
}
