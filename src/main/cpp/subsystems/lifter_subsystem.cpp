/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/lifter_subsystem.h"

#include "argos_lib/config/config_types.h"
#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"
#include "units/time.h"

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
    , m_armExtension{GetCANAddr(address::comp_bot::lifter::arm, address::practice_bot::lifter::arm, instance),
                     std::string(
                         GetCANBus(address::comp_bot::lifter::arm, address::practice_bot::lifter::arm, instance))}
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
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::arm, motorConfig::practice_bot::lifter::arm>(
      m_armExtension, 100_ms, instance);

  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::wrist,
                                         motorConfig::practice_bot::lifter::wrist>(m_wrist, 100_ms, instance);

  // Make back shoulder motor follow front shoulder motor
  m_shoulderFollower.Follow(m_shoulderLeader);
}

// This method will be called once per scheduler run
void LifterSubsystem::Periodic() {}
