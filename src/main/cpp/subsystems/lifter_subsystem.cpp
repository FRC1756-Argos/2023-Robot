/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/lifter_subsystem.h"

#include <argos_lib/config/cancoder_config.h>
#include <argos_lib/config/config_types.h>
#include <argos_lib/config/falcon_config.h>
#include <argos_lib/general/swerve_utils.h>
#include <constants/addresses.h>
#include <constants/encoders.h>
#include <constants/measure_up.h>
#include <constants/motors.h>
#include <units/time.h>
#include <utils/sensor_conversions.h>

#include "Constants.h"

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
    , m_shoulderEncoder{GetCANAddr(address::comp_bot::encoders::shoulderEncoder,
                                   address::practice_bot::encoders::shoulderEncoder,
                                   instance),
                        std::string(GetCANBus(address::comp_bot::encoders::shoulderEncoder,
                                              address::practice_bot::encoders::shoulderEncoder,
                                              instance))}
    , m_wristEncoder{GetCANAddr(address::comp_bot::encoders::wristEncoder,
                                address::practice_bot::encoders::wristEncoder,
                                instance),
                     std::string(GetCANBus(address::comp_bot::encoders::wristEncoder,
                                           address::practice_bot::encoders::wristEncoder,
                                           instance))}
    , m_shoulderHomeStorage{paths::shoulderHome}
    , m_wristHomingStorage{paths::wristHomesPath}
    , m_extensionTuner{"argos/lifter/extension",
                       {&m_armExtensionMotor},
                       0,
                       {
                           argos_lib::GetSensorConversionFactor(sensor_conversions::lifter::arm_extension::ToExtension),
                           1.0,
                           argos_lib::GetSensorConversionFactor(sensor_conversions::lifter::arm_extension::ToExtension),
                       }}
    , m_shoulderHomed{false}
    , m_extensionHomed{false}
    , m_wristHomed{false}
    , m_shoulderManualOverride{false}
    , m_extensionManualOverride{false}
    , m_wristManualOverride{false} {
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

  bool wristSuccess =
      argos_lib::cancoder_config::CanCoderConfig<encoder_conf::comp_bot::wristEncoder>(m_wristEncoder, 100_ms);
  if (!wristSuccess) {
    std::printf("{CRITICAL ERROR}%d Wirst encoder configuration failed\n", __LINE__);
  } else {
    InitializeWristHomes();
  }

  // Make back shoulder motor follow front shoulder motor
  m_shoulderFollower.Follow(m_shoulderLeader);

  bool shoulderSuccess = argos_lib::cancoder_config::CanCoderConfig<encoder_conf::comp_bot::shoulderEncoderConf>(
      m_shoulderEncoder, 100_ms);

  if (!shoulderSuccess) {
    std::printf("ERROR%d Shoulder encoder configuration failed, shoulder not homed\n", __LINE__);
    m_shoulderHomed = false;
  } else {
    InitializeShoulderHome();
  }
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

void LifterSubsystem::SetArmExtension(units::inch_t extension) {
  if (IsArmExtensionHomed()) {
    m_armExtensionMotor.Set(phoenix::motorcontrol::ControlMode::Position,
                            sensor_conversions::lifter::arm_extension::ToSensorUnit(extension));
  }
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

bool LifterSubsystem::IsShoulderManualOverride() {
  return m_shoulderManualOverride;
}

void LifterSubsystem::SetShoulderManualOverride(bool overrideState) {
  m_shoulderManualOverride = overrideState;
}

bool LifterSubsystem::IsExtensionManualOverride() {
  return m_extensionManualOverride;
}

void LifterSubsystem::SetExtensionManualOverride(bool overrideState) {
  m_extensionManualOverride = overrideState;
}

bool LifterSubsystem::IsWristManualOverride() {
  return m_wristManualOverride;
}

void LifterSubsystem::SetWristManualOverride(bool overrideState) {
  m_wristManualOverride = overrideState;
}

// This method will be called once per scheduler run
void LifterSubsystem::Periodic() {}

void LifterSubsystem::Disable() {
  StopShoulder();
  StopArmExtension();
  StopWrist();
}

bool LifterSubsystem::IsArmExtensionMoving() {
  return std::abs(m_armExtensionMotor.GetSelectedSensorVelocity()) > 10;
}

void LifterSubsystem::UpdateArmExtensionHome() {
  m_armExtensionMotor.SetSelectedSensorPosition(
      sensor_conversions::lifter::arm_extension::ToSensorUnit(measure_up::lifter::arm_extension::homeExtension));
  m_extensionHomed = true;
}

void LifterSubsystem::InitializeWristHomes() {
  const std::optional<units::degree_t> wristHomes = m_wristHomingStorage.Load();
  if (wristHomes) {
    units::degree_t currentencoder = units::make_unit<units::degree_t>(m_wristEncoder.GetAbsolutePosition());

    units::degree_t calcValue = currentencoder - wristHomes.value();

    m_wristEncoder.SetPosition(calcValue.to<double>());

    m_wristHomed = true;
  } else {
    m_wristHomed = false;
  }
}

void LifterSubsystem::UpdateWristHome() {
  const auto homeAngle = measure_up::lifter::wrist::homeAngle;
  units::degree_t currentEncoder = units::make_unit<units::degree_t>(m_wristEncoder.GetAbsolutePosition());
  bool saved = m_wristHomingStorage.Save(argos_lib::swerve::ConstrainAngle(currentEncoder - homeAngle, 0_deg, 360_deg));
  if (!saved) {
    std::printf("[CRITICAL ERROR]%d Wrist homes failed to save to to file system\n", __LINE__);
    m_wristHomed = false;
    return;
  }

  ErrorCode rslt = m_wristEncoder.SetPosition(homeAngle.to<double>());
  if (rslt != ErrorCode::OKAY) {
    std::printf("[CRITICAL ERROR]%d Error code %d returned by wristEncoder on position set attempt\n", __LINE__, rslt);
    m_wristHomed = false;
    return;
  }
  m_wristHomed = true;
}

void LifterSubsystem::InitializeShoulderHome() {
  const std::optional<units::degree_t> curHome = m_shoulderHomeStorage.Load();  // Try to load homes from fs
  if (curHome) {                                                                // if homes found
    units::degree_t curEncoder = units::make_unit<units::degree_t>(m_shoulderEncoder.GetAbsolutePosition());
    units::degree_t newPosition = curEncoder - curHome.value();

    // Error check
    ErrorCode rslt = m_shoulderEncoder.SetPosition(newPosition.to<double>(), 10);
    if (rslt != ErrorCode::OKAY) {
      std::printf("[CRITICAL ERROR]%d Error code %d returned by shoulderEncoder on home set attempt\n", __LINE__, rslt);
      m_shoulderHomed = false;
    } else {
      m_shoulderHomed = true;
    }

  } else {  // if homes not found
    std::printf("[CRITICAL ERROR]%d Homes were unable to be initialized\n", __LINE__);
    m_shoulderHomed = false;
  }
}

void LifterSubsystem::UpdateShoulderHome() {
  // save current position as home
  const auto homeAngle = measure_up::lifter::shoulder::homeAngle;
  const units::degree_t curEncoder = units::make_unit<units::degree_t>(m_shoulderEncoder.GetAbsolutePosition());
  bool saved = m_shoulderHomeStorage.Save(argos_lib::swerve::ConstrainAngle(curEncoder - homeAngle, 0_deg, 360_deg));
  if (!saved) {
    std::printf("[CRITICAL ERROR]%d Shoulder homes failed to save to file system\n", __LINE__);
    m_shoulderHomed = false;
    return;
  }

  ErrorCode rslt = m_shoulderEncoder.SetPosition(homeAngle.to<double>());
  if (rslt != ErrorCode::OKAY) {
    std::printf(
        "[CRITICAL ERROR]%d Error code %d returned by shoulderEncoder on position set attempt\n", __LINE__, rslt);
    m_shoulderHomed = false;
    return;
  }

  m_shoulderHomed = true;
}

bool LifterSubsystem::IsArmExtensionHomed() {
  return m_extensionHomed;
}
