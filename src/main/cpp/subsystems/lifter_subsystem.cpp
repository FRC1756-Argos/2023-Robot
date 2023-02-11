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
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>
#include <utils/sensor_conversions.h>

#include "Constants.h"
#include "argos_lib/config/cancoder_config.h"
#include "argos_lib/config/config_types.h"
#include "argos_lib/config/falcon_config.h"
#include "argos_lib/general/swerve_utils.h"
#include "constants/addresses.h"
#include "constants/encoders.h"
#include "constants/motors.h"
#include "units/time.h"
#include "utils/sensor_conversions.h"

/* ——————————————————— ARM SUBSYSTEM MEMBER FUNCTIONS —————————————————— */

LifterSubsystem::LifterSubsystem(argos_lib::RobotInstance instance)
    : m_shoulderDrive{GetCANAddr(address::comp_bot::lifter::frontShoulder,
                                 address::practice_bot::lifter::frontShoulder,
                                 instance),
                      std::string(GetCANBus(address::comp_bot::lifter::frontShoulder,
                                            address::practice_bot::lifter::frontShoulder,
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
    , m_kinematics{measure_up::lifter::fulcrumPosition,
                   measure_up::lifter::armBar::centerOfRotDis,
                   measure_up::lifter::effector::effectorFromArm}
    , m_shoulderHomeStorage{"homes/shoulderHome"}
    , m_wristHomingStorage{paths::wristHomesPath}
    , m_extensionTuner{"argos/lifter/extension",
                       {&m_armExtensionMotor},
                       0,
                       {
                           argos_lib::GetSensorConversionFactor(sensor_conversions::lifter::arm_extension::ToExtension),
                           1.0,
                           argos_lib::GetSensorConversionFactor(sensor_conversions::lifter::arm_extension::ToExtension),
                       }}
    , m_wristTuner{"argos/wristTune",
                   {&m_wrist},
                   0,
                   argos_lib::ClosedLoopSensorConversions{
                       argos_lib::GetSensorConversionFactor(sensor_conversions::lifter::wrist::ToAngle),
                       argos_lib::GetSensorConversionFactor(sensor_conversions::lifter::wrist::ToVelocity),
                       argos_lib::GetSensorConversionFactor(sensor_conversions::lifter::wrist::ToAngle)}}
    , m_shoulderTuner{"argos/shoulderTune",
                      {&m_shoulderDrive},
                      0,
                      argos_lib::ClosedLoopSensorConversions{
                          argos_lib::GetSensorConversionFactor(sensor_conversions::lifter::shoulder::ToAngle),
                          1.0,
                          argos_lib::GetSensorConversionFactor(sensor_conversions::lifter::shoulder::ToAngle)}}
    , m_shoulderHomed{false}
    , m_extensionHomed{false}
    , m_wristHomed{false}
    , m_shoulderManualOverride{false}
    , m_extensionManualOverride{false}
    , m_wristManualOverride{false} {
  /* ———————————————————————— MOTOR CONFIGURATION ———————————————————————— */

  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::shoulderDrive,
                                         motorConfig::practice_bot::lifter::shoulderDrive>(
      m_shoulderDrive, 100_ms, instance);
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
  m_shoulderDrive.Set(phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void LifterSubsystem::StopShoulder() {
  m_shoulderDrive.SetNeutralMode(phoenix::motorcontrol::NeutralMode::Brake);
  m_shoulderDrive.Set(0.0);
}

void LifterSubsystem::SetArmExtensionSpeed(double speed) {
  m_armExtensionMotor.Set(phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void LifterSubsystem::SetArmExtension(units::inch_t extension) {
  if (!IsArmExtensionHomed()) {
    std::cerr << "ARGOS_ERR Arm extension commanded while not home\n";
    return;
  }

  SetExtensionManualOverride(false);

  // guard against out of bounds functions
  if (extension < measure_up::lifter::arm_extension::minExtension) {
    extension = measure_up::lifter::arm_extension::minExtension;
  } else if (extension > measure_up::lifter::arm_extension::maxExtension) {
    extension = measure_up::lifter::arm_extension::maxExtension;
  }

  m_armExtensionMotor.Set(phoenix::motorcontrol::ControlMode::Position,
                          sensor_conversions::lifter::arm_extension::ToSensorUnit(extension));
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

void LifterSubsystem::SetWristAngle(units::degree_t wristAngle) {
  // REMOVEME
  frc::SmartDashboard::PutNumber("Set Wrist Angle Degrees Setpoint", wristAngle.to<double>());

  if (!m_wristHomed) {
    Disable();
    return;
  }

  SetWristManualOverride(false);

  if (wristAngle < measure_up::lifter::wrist::minAngle) {
    wristAngle = measure_up::lifter::wrist::minAngle;
  } else if (wristAngle > measure_up::lifter::wrist::maxAngle) {
    wristAngle = measure_up::lifter::wrist::maxAngle;
  }

  frc::SmartDashboard::PutNumber("Set Wrist Angle Degrees Setpoint (SensorUnits)",
                                 sensor_conversions::lifter::wrist::ToSensorUnit(wristAngle));

  m_wrist.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
              sensor_conversions::lifter::wrist::ToSensorUnit(wristAngle));
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
  EnableArmExtensionSoftLimits();
}

void LifterSubsystem::InitializeWristHomes() {
  const std::optional<units::degree_t> wristHomes = m_wristHomingStorage.Load();
  if (wristHomes) {
    units::degree_t currentencoder = units::make_unit<units::degree_t>(m_wristEncoder.GetAbsolutePosition());

    units::degree_t calcValue =
        argos_lib::swerve::ConstrainAngle(currentencoder - wristHomes.value(), -180_deg, 180_deg);

    m_wristEncoder.SetPosition(calcValue.to<double>());

    m_wristHomed = true;

    EnableWristSoftLimits();
  } else {
    m_wristHomed = false;
  }
}

void LifterSubsystem::UpdateWristHome() {
  const auto homeAngle = measure_up::lifter::wrist::homeAngle;
  units::degree_t currentEncoder = units::make_unit<units::degree_t>(m_wristEncoder.GetAbsolutePosition());
  auto valToSave = argos_lib::swerve::ConstrainAngle(currentEncoder - homeAngle, -180_deg, 180_deg);
  bool saved = m_wristHomingStorage.Save(valToSave);
  if (!saved) {
    std::printf("[CRITICAL ERROR]%d Wrist homes failed to save to to file system\n", __LINE__);
    m_wristHomed = false;
    return;
  }

  InitializeWristHomes();
}

void LifterSubsystem::InitializeShoulderHome() {
  const std::optional<units::degree_t> curHome = m_shoulderHomeStorage.Load();  // Try to load homes from fs
  if (curHome) {                                                                // if homes found
    units::degree_t curEncoder = units::make_unit<units::degree_t>(m_shoulderEncoder.GetAbsolutePosition());
    units::degree_t newPosition = argos_lib::swerve::ConstrainAngle(curEncoder - curHome.value(), -180_deg, 180_deg);

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

  EnableShoulderSoftLimits();
}

void LifterSubsystem::UpdateShoulderHome() {
  // save current position as home
  const auto homeAngle = measure_up::lifter::shoulder::homeAngle;
  const units::degree_t curEncoder = units::make_unit<units::degree_t>(m_shoulderEncoder.GetAbsolutePosition());
  bool saved = m_shoulderHomeStorage.Save(argos_lib::swerve::ConstrainAngle(curEncoder - homeAngle, -180_deg, 180_deg));
  if (!saved) {
    std::printf("[CRITICAL ERROR]%d Shoulder homes failed to save to file system\n", __LINE__);
    m_shoulderHomed = false;
    return;
  }

  m_shoulderHomed = true;

  InitializeShoulderHome();
}

void LifterSubsystem::SetShoulderAngle(units::degree_t angle) {
  if (!m_shoulderHomed) {  // If shoulder is not homed, return and do nothing
    return;
  }

  SetShoulderManualOverride(false);

  angle = argos_lib::swerve::ConstrainAngle(angle, -180_deg, 180_deg);  // Constrain to 0 to 360

  if (angle < measure_up::lifter::shoulder::minAngle) {  // Handle angle below bound, clamp to min
    angle = measure_up::lifter::shoulder::minAngle;
    std::printf("ERROR Shoulder commanded to angle below min of [%f]\n",
                measure_up::lifter::shoulder::minAngle.to<double>());
  } else if (angle > measure_up::lifter::shoulder::maxAngle) {  // Handle angle above bound, clamp to max
    angle = measure_up::lifter::shoulder::maxAngle;
    std::printf("ERROR Shoulder commanded to angle above bound of [%f]\n",
                measure_up::lifter::shoulder::maxAngle.to<double>());
  }

  m_shoulderDrive.Set(motorcontrol::ControlMode::Position, sensor_conversions::lifter::shoulder::ToSensorUnit(angle));
}

frc::Translation2d LifterSubsystem::GetArmPose() {
  LifterPosition lfPose = GetLifterPosition();
  return m_kinematics.GetPose(ArmState{lfPose.state});
}

bool LifterSubsystem::IsArmExtensionHomed() {
  return m_extensionHomed;
}

units::degree_t LifterSubsystem::GetWristAngle() {
  return sensor_conversions::lifter::wrist::ToAngle(m_wrist.GetSelectedSensorPosition());
}
units::inch_t LifterSubsystem::GetArmExtension() {
  return sensor_conversions::lifter::arm_extension::ToExtension(m_armExtensionMotor.GetSelectedSensorPosition());
}
units::degree_t LifterSubsystem::GetShoulderAngle() {
  return sensor_conversions::lifter::shoulder::ToAngle(m_shoulderDrive.GetSelectedSensorPosition());
}
LifterSubsystem::LifterPosition LifterSubsystem::GetLifterPosition() {
  return {GetWristAngle(), ArmState{GetArmExtension(), GetShoulderAngle()}};
}

void LifterSubsystem::EnableWristSoftLimits() {
  if (!m_wristHomed) {
    std::cerr << "ARGOS_ERROR Attempted to enable wrist soft limits without homes\n";
    return;
  }
  m_wrist.ConfigForwardSoftLimitThreshold(
      sensor_conversions::lifter::wrist::ToSensorUnit(measure_up::lifter::wrist::maxAngle));
  m_wrist.ConfigReverseSoftLimitThreshold(
      sensor_conversions::lifter::wrist::ToSensorUnit(measure_up::lifter::wrist::minAngle));
  m_wrist.ConfigForwardSoftLimitEnable(true);
  m_wrist.ConfigReverseSoftLimitEnable(true);
}

void LifterSubsystem::DisableWristSoftLimits() {
  m_wrist.ConfigForwardSoftLimitEnable(false);
  m_wrist.ConfigReverseSoftLimitEnable(false);
}

void LifterSubsystem::EnableArmExtensionSoftLimits() {
  if (!m_extensionHomed) {
    std::cerr << "ARGOS_ERROR Attempted to enable extension soft limits without homes\n";
    return;
  }
  m_armExtensionMotor.ConfigForwardSoftLimitThreshold(
      sensor_conversions::lifter::arm_extension::ToSensorUnit(measure_up::lifter::arm_extension::maxExtension));
  m_armExtensionMotor.ConfigReverseSoftLimitThreshold(
      sensor_conversions::lifter::arm_extension::ToSensorUnit(measure_up::lifter::arm_extension::minExtension));
  m_armExtensionMotor.ConfigForwardSoftLimitEnable(true);
  m_armExtensionMotor.ConfigReverseSoftLimitEnable(true);
}

void LifterSubsystem::DisableArmExtensionSoftLimits() {
  m_armExtensionMotor.ConfigForwardSoftLimitEnable(false);
  m_armExtensionMotor.ConfigReverseSoftLimitEnable(false);
}

void LifterSubsystem::EnableShoulderSoftLimits() {
  if (!m_shoulderHomed) {
    std::cerr << "ARGOS_ERROR Attempted to enable shoulder soft limits without homes\n";
    return;
  }
  m_shoulderDrive.ConfigForwardSoftLimitThreshold(
      sensor_conversions::lifter::shoulder::ToSensorUnit(measure_up::lifter::shoulder::maxAngle));
  m_shoulderDrive.ConfigReverseSoftLimitThreshold(
      sensor_conversions::lifter::shoulder::ToSensorUnit(measure_up::lifter::shoulder::minAngle));
  m_shoulderDrive.ConfigForwardSoftLimitEnable(true);
  m_shoulderDrive.ConfigReverseSoftLimitEnable(true);
}

void LifterSubsystem::DisableShoulderSoftLimits() {
  m_shoulderDrive.ConfigForwardSoftLimitEnable(false);
  m_shoulderDrive.ConfigReverseSoftLimitEnable(false);
}
