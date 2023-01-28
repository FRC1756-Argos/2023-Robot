/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/home_arm_extension_command.h"

#include <chrono>

using namespace std::chrono_literals;

HomeArmExtensionCommand::HomeArmExtensionCommand(LifterSubsystem& subsystem) : m_LifterSubsystem(subsystem) {
  // TODO //put stuff here
}

void HomeArmExtensionCommand::Initialize() {
  m_LifterSubsystem.SetArmExtensionSpeed(-0.07);
  m_startTime = std::chrono::steady_clock::now();
}

void HomeArmExtensionCommand::Execute() {
  if (m_LifterSubsystem.IsManualOverride() || (std::chrono::steady_clock::now() - m_startTime) > 2.0s) {
    Cancel();
  } else {
    m_LifterSubsystem.SetArmExtensionSpeed(-0.07);
  }
}

void HomeArmExtensionCommand::End(bool interrupted) {
  if (!interrupted) {
    m_LifterSubsystem.UpdateHookHome();
  }
  m_LifterSubsystem.SetArmExtensionSpeed(0.0);
}

bool HomeArmExtensionCommand::IsFinished() {
  return !m_hookMovingDebounce(m_LifterSubsystem.IsHookMoving());
}
