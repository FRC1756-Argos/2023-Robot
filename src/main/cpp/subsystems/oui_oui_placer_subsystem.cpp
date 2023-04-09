/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/oui_oui_placer_subsystem.h"

#include <argos_lib/config/falcon_config.h>
#include <argos_lib/general/angle_utils.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/math.h>

#include <string>

#include "constants/addresses.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "utils/sensor_conversions.h"

OuiOuiPlacerSubsystem::OuiOuiPlacerSubsystem(argos_lib::RobotInstance instance)
    : m_instance{instance}
    , m_manualOverride{false}
    , m_ouiOuiDrive{GetCANAddr(address::comp_bot::oui_oui_placer::ouiOuiDriver,
                               address::practice_bot::ouiOuiDriver::ouiOuiDriver,
                               instance),
                    std::string(GetCANBus(address::comp_bot::oui_oui_placer::ouiOuiDriver,
                                          address::practice_bot::ouiOuiDriver::ouiOuiDriver,
                                          instance))}
    , m_ouiOuiTuner{"argos/ouiOuiDrive",
                    {&m_ouiOuiDrive},
                    0,
                    {argos_lib::GetSensorConversionFactor(sensor_conversions::oui_oui_place::ToAngle),
                     argos_lib::GetSensorConversionFactor(sensor_conversions::oui_oui_place::ToVelocity),
                     argos_lib::GetSensorConversionFactor(sensor_conversions::oui_oui_place::ToAngle)}}
    , m_stallDebounce{argos_lib::DebounceSettings{.activateTime = 200_ms, .clearTime = 0_ms}}
    , m_stalled{false}
    , m_softLimitsEnabled{true} {
  // Configure oui oui motor
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::oui_oui_placer,
                                         motorConfig::practice_bot::oui_oui_placer>(m_ouiOuiDrive, 100_ms, instance);
  m_ouiOuiDrive.SetSelectedSensorPosition(
      sensor_conversions::oui_oui_place::ToSensorUnit(measure_up::oui_oui_place::maxAngle));
  EnablePlacerSoftLimits();
}
void OuiOuiPlacerSubsystem::SetOuiOuiSpeed(double percentOutput, bool overrideSoftLimits) {
  double maxForwardOutput = 1.0;
  double maxReverseOutput = -1.0;

  if (overrideSoftLimits == m_softLimitsEnabled) {
    m_softLimitsEnabled = !overrideSoftLimits;
    m_ouiOuiDrive.OverrideSoftLimitsEnable(!overrideSoftLimits);
  }

  if (ForwardSoftLimitExceeded()) {
    maxForwardOutput = 0.25;
  } else if (ReverseSoftLimitExceeded()) {
    maxReverseOutput = -0.25;
  }
  double clampedOutput = std::clamp<double>(percentOutput, maxReverseOutput, maxForwardOutput);  // Enforce bounds
  m_ouiOuiDrive.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, clampedOutput);
}

void OuiOuiPlacerSubsystem::StopOuiOuiPlacer() {
  m_ouiOuiDrive.Set(0);
}

units::degree_t OuiOuiPlacerSubsystem::GetOuiOuiAngle() {
  return sensor_conversions::oui_oui_place::ToAngle(m_ouiOuiDrive.GetSelectedSensorPosition());
}

void OuiOuiPlacerSubsystem::EnablePlacerSoftLimits() {
  m_ouiOuiDrive.ConfigForwardSoftLimitThreshold(
      sensor_conversions::oui_oui_place::ToSensorUnit(measure_up::oui_oui_place::maxAngle));
  m_ouiOuiDrive.ConfigReverseSoftLimitThreshold(
      sensor_conversions::oui_oui_place::ToSensorUnit(measure_up::oui_oui_place::minAngle));
  m_ouiOuiDrive.ConfigForwardSoftLimitEnable(true);
  m_ouiOuiDrive.ConfigReverseSoftLimitEnable(true);
}

void OuiOuiPlacerSubsystem::DisablePlacerSoftLimits() {
  m_ouiOuiDrive.ConfigForwardSoftLimitEnable(false);
  m_ouiOuiDrive.ConfigReverseSoftLimitEnable(false);
}

bool OuiOuiPlacerSubsystem::IsStalled() {
  return m_stalled;
}

bool OuiOuiPlacerSubsystem::ForwardSoftLimitExceeded() {
  return sensor_conversions::oui_oui_place::ToAngle(m_ouiOuiDrive.GetSelectedSensorPosition()) >
         measure_up::oui_oui_place::maxAngle;
}

bool OuiOuiPlacerSubsystem::ReverseSoftLimitExceeded() {
  return sensor_conversions::oui_oui_place::ToAngle(m_ouiOuiDrive.GetSelectedSensorPosition()) <
         measure_up::oui_oui_place::minAngle;
}

// This method will be called once per scheduler run
void OuiOuiPlacerSubsystem::Periodic() {
  auto statorCurrentAmplitude = units::math::abs(units::current::ampere_t{m_ouiOuiDrive.GetStatorCurrent()});
  auto velocityAmplitude =
      units::math::abs(sensor_conversions::oui_oui_place::ToVelocity(m_ouiOuiDrive.GetSelectedSensorVelocity()));
  m_stalled = m_stallDebounce(statorCurrentAmplitude > 15_A && velocityAmplitude < 10_deg_per_s);

  frc::SmartDashboard::PutNumber("OuiOuiPlacer/StatorCurrentAmplitude", statorCurrentAmplitude.to<double>());
  frc::SmartDashboard::PutNumber("OuiOuiPlacer/VelocityAmplitude", velocityAmplitude.to<double>());
  frc::SmartDashboard::PutBoolean("OuiOuiPlacer/Stalled", m_stalled);
}
