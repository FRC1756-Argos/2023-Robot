/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/bash_guard_subsystem.h"

#include <argos_lib/config/falcon_config.h>
#include <constants/measure_up.h>
#include <utils/sensor_conversions.h>

#include "constants/addresses.h"
#include "constants/motors.h"

BashGuardSubsystem::BashGuardSubsystem(argos_lib::RobotInstance instance)
    : m_bashGuard{
          GetCANAddr(address::comp_bot::bash_guard::extension, address::practice_bot::bash_guard::extension, instance),
          std::string(GetCANBus(
              address::comp_bot::bash_guard::extension, address::practice_bot::bash_guard::extension, instance))} {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::bash_guard::extension,
                                         motorConfig::practice_bot::bash_guard::extension>(
      m_bashGuard, 100_ms, instance);
}

bool BashGuardSubsystem::IsBashGuardMoving() {
  return std::abs(m_bashGuard.GetSelectedSensorVelocity()) > 10;
}

void BashGuardSubsystem::SetBashGuardManualOverride(bool overrideState) {
  m_bashGuardManualOverride = overrideState;
}

bool BashGuardSubsystem::IsBashGuardManualOverride() {
  return m_bashGuardManualOverride;
}

void BashGuardSubsystem::UpdateBashGuardHome() {
  m_bashGuard.SetSelectedSensorPosition(sensor_conversions::bashguard::ToSensorUnit(measure_up::bash::homeExtension));
  m_bashguardHomed = true;
}

void BashGuardSubsystem::SetExtensionSpeed(double speed) {
  m_bashGuard.Set(phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

// This method will be called once per scheduler run
void BashGuardSubsystem::Periodic() {}
