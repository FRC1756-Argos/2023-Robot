/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/lifter_subsystem.h"

#include <argos_lib/config/config_types.h>
#include <argos_lib/config/falcon_config.h>
#include <constants/addresses.h>
#include <constants/measure_up.h>
#include <constants/motors.h>
#include <units/time.h>
#include <utils/sensor_conversions.h>

/* ——————————————————— ARM SUBSYSTEM MEMBER FUNCTIONS —————————————————— */

LifterSubsystem::LifterSubsystem(argos_lib::RobotInstance instance)
    : m_shoulderLeader{GetCANAddr(address::comp_bot::lifter::frontShoulder,
                                  address::practice_bot::lifter::frontShoulder,
                                  instance),
                       std::string(GetCANBus(address::comp_bot::lifter::frontShoulder,
                                             address::practice_bot::lifter::frontShoulder,
                                             instance))}
    , m_shoulderFollower{GetCANAddr(address::comp_bot::lifter::backShoulder,
                                    address::practice_bot::lifter::backShoulder,
                                    instance),
                         std::string(GetCANBus(address::comp_bot::lifter::backShoulder,
                                               address::practice_bot::lifter::backShoulder,
                                               instance))}
    , m_armExtensionMotor{GetCANAddr(address::comp_bot::lifter::armExtension,
                                     address::practice_bot::lifter::armExtension,
                                     instance),
                          std::string(GetCANBus(address::comp_bot::lifter::armExtension,
                                                address::practice_bot::lifter::armExtension,
                                                instance))}
    , m_wrist{GetCANAddr(address::comp_bot::lifter::wrist, address::practice_bot::lifter::wrist, instance),
              std::string(GetCANBus(address::comp_bot::lifter::wrist, address::practice_bot::lifter::wrist, instance))}
    , m_armExtensionEncoder{GetCANAddr(address::comp_bot::encoders::armExtenderEncoder,
                                       address::practice_bot::encoders::armExtenderEncoder,
                                       instance),
                            std::string(GetCANBus(address::comp_bot::encoders::armExtenderEncoder,
                                                  address::practice_bot::encoders::armExtenderEncoder,
                                                  instance))}
    , m_shoulderEncoder{GetCANAddr(address::comp_bot::encoders::shoulderEncoder,
                                   address::practice_bot::encoders::shoulderEncoder,
                                   instance),
                        std::string(GetCANBus(address::comp_bot::encoders::shoulderEncoder,
                                              address::practice_bot::encoders::shoulderEncoder,
                                              instance))}
    , m_wristEncoder{
          GetCANAddr(
              address::comp_bot::encoders::wristEncoder, address::practice_bot::encoders::wristEncoder, instance),
          std::string(GetCANBus(
              address::comp_bot::encoders::wristEncoder, address::practice_bot::encoders::wristEncoder, instance))} {
  /* ———————————————————————— MOTOR CONFIGURATION ———————————————————————— */

  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::shoulderLeader,
                                         motorConfig::practice_bot::lifter::shoulderLeader>(
      m_shoulderLeader, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::shoulderFollower,
                                         motorConfig::practice_bot::lifter::shoulderFollower>(
      m_shoulderFollower, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::armExtension,
                                         motorConfig::practice_bot::lifter::armExtension>(
      m_armExtensionMotor, 100_ms, instance);

  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::wrist,
                                         motorConfig::practice_bot::lifter::wrist>(m_wrist, 100_ms, instance);

  // Make back shoulder motor follow front shoulder motor
  m_shoulderFollower.Follow(m_shoulderLeader);
}

/* —————————————————— LifterSubsystem Member Functions ————————————————— */

void LifterSubsystem::SetShoulderSpeed(double speed) {
  m_shoulderLeader.Set(phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void LifterSubsystem::StopShoulder() {
  m_shoulderLeader.SetNeutralMode(phoenix::motorcontrol::NeutralMode::Brake);
  m_shoulderLeader.Set(0.0);
}

void LifterSubsystem::SetArmExtensionSpeed(double speed) {
  m_armExtensionMotor.Set(phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void LifterSubsystem::StopArmExtension() {
  m_armExtensionMotor.SetNeutralMode(phoenix::motorcontrol::NeutralMode::Brake);
  m_armExtensionMotor.Set(0.0);
}

void LifterSubsystem::SetWristSpeed(double speed) {
  m_wrist.Set(phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void LifterSubsystem::StopWrist() {
  m_wrist.SetNeutralMode(phoenix::motorcontrol::NeutralMode::Brake);
  m_wrist.Set(0.0);
}

// This method will be called once per scheduler run
void LifterSubsystem::Periodic() {}

void LifterSubsystem::Disable() {
  StopShoulder();
  StopArmExtension();
  StopWrist();
}

bool LifterSubsystem::IsArmMoving() {
  return std::abs(m_armExtensionMotor.GetSelectedSensorPosition()) > 10;
}

void LifterSubsystem::UpdateArmHome() {
  m_armExtensionMotor.SetSelectedSensorPosition(
      sensor_conversions::lifter::arm_extension::ToSensorUnit(measure_up::arm_extension::homeExtension));
}
