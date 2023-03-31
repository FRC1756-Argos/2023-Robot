/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/oui_oui_placer_subsystem.h"

#include <argos_lib/config/falcon_config.h>
#include <argos_lib/general/angle_utils.h>

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
                     argos_lib::GetSensorConversionFactor(sensor_conversions::oui_oui_place::ToAngle)}} {
  // Configure oui oui motor
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::oui_oui_placer,
                                         motorConfig::practice_bot::oui_oui_placer>(m_ouiOuiDrive, 100_ms, instance);
}

void OuiOuiPlacerSubsystem::TakeManualControl() {
  m_manualOverride = true;
}

void OuiOuiPlacerSubsystem::ReleaseManualControl() {
  m_manualOverride = false;
}

bool OuiOuiPlacerSubsystem::ReadManualControl() {
  return m_manualOverride;
}

// @todo validate this first, to get config right.
void OuiOuiPlacerSubsystem::SetOuiOuiSpeed(double percentOutput) {
  TakeManualControl();  // If we command using this function, there should be no closed loop control, right?
  std::clamp<double>(percentOutput, -1.0, 1.0);  // Enforce bounds
  m_ouiOuiDrive.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, percentOutput);
}

// @todo make sure to use motor tuner first before even dreaming of using this function
void OuiOuiPlacerSubsystem::SetOuiOuiAngle(units::degree_t angle) {
  if (ReadManualControl()) {
    return;  // Subsystem is manually overriden, do nothing
  }
  argos_lib::angle::ConstrainAngle(angle, 0_deg, 360_deg);
  // Bound to upper and lower angle limits
  std::clamp<units::degree_t>(angle, measure_up::oui_oui_place::minAngle, measure_up::oui_oui_place::maxAngle);
  // * Note units are misleading on accel config
  m_ouiOuiDrive.ConfigMotionAcceleration(sensor_conversions::oui_oui_place::ToSensorVelocity(5_deg_per_s));
  m_ouiOuiDrive.ConfigMotionCruiseVelocity(sensor_conversions::oui_oui_place::ToSensorVelocity(10_deg_per_s));
  m_ouiOuiDrive.Set(phoenix::motorcontrol::ControlMode::MotionMagic,
                    sensor_conversions::oui_oui_place::ToSensorUnit(angle));
}

// This method will be called once per scheduler run
void OuiOuiPlacerSubsystem::Periodic() {}
