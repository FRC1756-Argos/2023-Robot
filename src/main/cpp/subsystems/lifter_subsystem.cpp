/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/lifter_subsystem.h"

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
    , m_armExtension{GetCANAddr(address::comp_bot::lifter::armExtension,
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
    , m_wristEncoder{GetCANAddr(address::comp_bot::encoders::wristEncoder,
                                address::practice_bot::encoders::wristEncoder,
                                instance),
                     std::string(GetCANBus(address::comp_bot::encoders::wristEncoder,
                                           address::practice_bot::encoders::wristEncoder,
                                           instance))}
    , m_wristHomingStorage{paths::wristHomesPath}
    , m_wristHomed{false}
    , m_shoulderHomeStorage{paths::shoulderHome}
    , m_shoulderHomed{false} {
  /* ———————————————————————— MOTOR CONFIGURATION ———————————————————————— */

  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::shoulderLeader,
                                         motorConfig::practice_bot::lifter::shoulderLeader>(
      m_shoulderLeader, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::shoulderFollower,
                                         motorConfig::practice_bot::lifter::shoulderFollower>(
      m_shoulderFollower, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::armExtension,
                                         motorConfig::practice_bot::lifter::armExtension>(
      m_armExtension, 100_ms, instance);

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

void LifterSubsystem::StopArm() {
  m_shoulderLeader.SetNeutralMode(phoenix::motorcontrol::NeutralMode::Brake);
  m_shoulderLeader.Set(0.0);
}

void LifterSubsystem::SetArmExtensionSpeed(double speed) {
  m_armExtension.Set(phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void LifterSubsystem::StopArmExtension() {
  m_armExtension.SetNeutralMode(phoenix::motorcontrol::NeutralMode::Brake);
  m_armExtension.Set(0.0);
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
  StopArm();
  StopArmExtension();
  StopWrist();
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

void LifterSubsystem::SetShoulderAngle(units::degree_t angle) {
  if (!m_shoulderHomed) {  // If shoulder is not homed, return and do nothing
    return;
  }

  angle = argos_lib::swerve::ConstrainAngle(angle, 0_deg, 360_deg);  // Constrain to 0 to 360

  if (angle < measure_up::lifter::shoulder::minAngle) {  // Handle angle below bound, clamp to min
    angle = measure_up::lifter::shoulder::minAngle;
    std::printf("ERROR Shoulder commanded to angle below min of [%f]\n",
                measure_up::lifter::shoulder::minAngle.to<double>());
  } else if (angle > measure_up::lifter::shoulder::maxAngle) {  // Handle angle above bound, clamp to max
    angle = measure_up::lifter::shoulder::maxAngle;
    std::printf("ERROR Shoulder commanded to angle above bound of [%f]\n",
                measure_up::lifter::shoulder::maxAngle.to<double>());
  }

  m_shoulderLeader.Set(motorcontrol::ControlMode::Position, sensor_conversions::lifter::shoulder::ToSensorUnit(angle));
}

units::degree_t LifterSubsystem::GetShoulderAngle() {
  return units::make_unit<units::degree_t>(m_shoulderEncoder.GetPosition());
}

units::inch_t LifterSubsystem::GetArmLen() {
  return sensor_conversions::lifter::armExtension::ToExtension(m_armExtensionEncoder.GetAbsolutePosition());
}

frc::Translation2d LifterSubsystem::GetEffectorPos(LifterState state) {
  LifterKinematics::GetPose(
      state,
      frc::Translation2d{state.armLen + measure_up::lifter::effector::xDisFromArmEnd,
                         measure_up::lifter::armBar::centerOfRotDis + measure_up::lifter::effector::yDisFromArmEnd});
}

frc::Translation2d LifterSubsystem::GetArmEndPos(LifterState state) {
  LifterKinematics::GetPose(state, frc::Translation2d{state.armLen, measure_up::lifter::armBar::centerOfRotDis});
}

LifterState LifterSubsystem::GetLifterState(frc::Translation2d desiredPose, bool effector) {
  units::inch_t effectorDisFromCenter;
  if (effector) {
    effectorDisFromCenter = measure_up::lifter::armBar::centerOfRotDis + measure_up::lifter::effector::yDisFromArmEnd;
  } else {
    effectorDisFromCenter = measure_up::lifter::armBar::centerOfRotDis;
  }

  LifterState state = LifterKinematics::GetJoints(desiredPose, effectorDisFromCenter);
}
