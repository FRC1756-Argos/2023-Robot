/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/bash_guard_subsystem.h"

#include <argos_lib/config/falcon_config.h>
#include <constants/measure_up.h>
#include <utils/sensor_conversions.h>

#include "constants/addresses.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "utils/sensor_conversions.h"

BashGuardSubsystem::BashGuardSubsystem(argos_lib::RobotInstance instance)
    : m_bashGuard{GetCANAddr(
                      address::comp_bot::bash_guard::extension, address::practice_bot::bash_guard::extension, instance),
                  std::string(GetCANBus(address::comp_bot::bash_guard::extension,
                                        address::practice_bot::bash_guard::extension,
                                        instance))}
    , m_bashTuner{"argos/bashTuner",
                  {&m_bashGuard},
                  0,
                  argos_lib::ClosedLoopSensorConversions{
                      argos_lib::GetSensorConversionFactor(sensor_conversions::bashguard::ToExtension),
                      argos_lib::GetSensorConversionFactor(sensor_conversions::bashguard::ToVelocity),
                      argos_lib::GetSensorConversionFactor(sensor_conversions::bashguard::ToExtension)}} {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::bash_guard::extension,
                                         motorConfig::practice_bot::bash_guard::extension>(
      m_bashGuard, 100_ms, instance);
}

bool BashGuardSubsystem::IsBashGuardMoving() {
  return std::abs(m_bashGuard.GetSelectedSensorVelocity()) > 10;
}

bool BashGuardSubsystem::IsBashGuardMPComplete() {
  return m_bashGuard.IsMotionProfileFinished();
}
ctre::phoenix::motion::BufferedTrajectoryPointStream& BashGuardSubsystem::GetMPStream() {
  return m_bashStream;
}

void BashGuardSubsystem::StartMotionProfile(size_t streamSize) {
  SetBashGuardManualOverride(false);
  m_bashGuard.StartMotionProfile(
      m_bashStream, std::min<size_t>(10, streamSize), ctre::phoenix::motorcontrol::ControlMode::MotionProfile);
}

void BashGuardSubsystem::Disable() {
  m_bashGuard.Set(0.0);
  m_bashStream.Clear();
}

void BashGuardSubsystem::Stop() {
  m_bashGuard.Set(0.0);
}

int BashGuardSubsystem::GetMotorMPBufferCount() {
  return m_bashGuard.GetMotionProfileTopLevelBufferCount();
}

void BashGuardSubsystem::SetExtensionLength(units::inch_t length) {
  if (!IsBashGuardHomed()) {
    return;
  }

  SetBashGuardManualOverride(false);
  length = std::clamp<units::inch_t>(length, measure_up::bash::minExtension, measure_up::bash::maxExtension);
  m_bashGuard.Set(phoenix::motorcontrol::ControlMode::Position, sensor_conversions::bashguard::ToSensorUnit(length));
}

bool BashGuardSubsystem::IsBashGuardHomed() {
  return m_bashGuardHomed;
}

void BashGuardSubsystem::SetBashGuardManualOverride(bool overrideState) {
  m_bashGuardManualOverride = overrideState;
}

units::inch_t BashGuardSubsystem::GetBashGuardExtension() {
  return sensor_conversions::bashguard::ToExtension(m_bashGuard.GetSelectedSensorPosition());
}

bool BashGuardSubsystem::IsBashGuardManualOverride() {
  return m_bashGuardManualOverride;
}

void BashGuardSubsystem::UpdateBashGuardHome() {
  m_bashGuard.SetSelectedSensorPosition(sensor_conversions::bashguard::ToSensorUnit(measure_up::bash::homeExtension));
  m_bashGuardHomed = true;
  EnableBashGuardSoftLimits();
}

void BashGuardSubsystem::SetExtensionSpeed(double speed) {
  m_bashGuard.Set(phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

// This method will be called once per scheduler run
void BashGuardSubsystem::Periodic() {}

void BashGuardSubsystem::EnableBashGuardSoftLimits() {
  if (m_bashGuardHomed) {
    m_bashGuard.ConfigForwardSoftLimitThreshold(
        sensor_conversions::bashguard::ToSensorUnit(measure_up::bash::maxExtension));
    m_bashGuard.ConfigReverseSoftLimitThreshold(
        sensor_conversions::bashguard::ToSensorUnit(measure_up::bash::minExtension));
    m_bashGuard.ConfigForwardSoftLimitEnable(true);
    m_bashGuard.ConfigReverseSoftLimitEnable(true);
  }
}
void BashGuardSubsystem::DisableBashGuardSoftLimits() {
  m_bashGuard.ConfigForwardSoftLimitEnable(false);
  m_bashGuard.ConfigReverseSoftLimitEnable(false);
}
