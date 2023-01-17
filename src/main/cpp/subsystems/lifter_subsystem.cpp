/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/lifter_subsystem.h"

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"
#include "units/time.h"

/* ——————————————————— ARM SUBSYSTEM MEMBER FUNCTIONS —————————————————— */

lifter_subsystem::lifter_subsystem(argos_lib::RobotInstance instance)
    : m_frontShoulder{GetCANAddr(address::comp_bot::lifter::frontShoulder,
                                 address::practice_bot::lifter::frontShoulder,
                                 instance),
                      GetCANBus(address::comp_bot::lifter::frontShoulder,
                                address::practice_bot::lifter::frontShoulder,
                                instance)}
    , m_backShoulder{GetCANAddr(address::comp_bot::lifter::backShoulder,
                                address::practice_bot::lifter::backShoulder,
                                instance),
                     GetCANBus(address::comp_bot::lifter::backShoulder,
                               address::practice_bot::lifter::backShoulder,
                               instance)}
    , m_arm{GetCANAddr(address::comp_bot::lifter::arm, address::practice_bot::lifter::arm, instance),
            GetCANBus(address::comp_bot::lifter::arm, address::practice_bot::lifter::arm, instance)}
    , m_wrist{GetCANAddr(address::comp_bot::lifter::wrist, address::practice_bot::lifter::wrist, instance),
              GetCANBus(address::comp_bot::lifter::wrist, address::practice_bot::lifter::wrist, instance)}
    , m_armEncoder{GetCANAddr(address::comp_bot::encoders::armExtenderEncoder,
                              address::practice_bot::encoders::armExtenderEncoder,
                              instance),
                   GetCANBus(address::comp_bot::encoders::armExtenderEncoder,
                             address::practice_bot::encoders::armExtenderEncoder,
                             instance)}
    , m_shoulderEncoder{GetCANAddr(address::comp_bot::encoders::shoulderEncoder,
                                   address::practice_bot::encoders::shoulderEncoder,
                                   instance),
                        GetCANBus(address::comp_bot::encoders::shoulderEncoder,
                                  address::practice_bot::encoders::shoulderEncoder,
                                  instance)}
    , m_wristEncoder{
          GetCANAddr(
              address::comp_bot::encoders::wristEncoder, address::practice_bot::encoders::wristEncoder, instance),
          GetCANBus(
              address::comp_bot::encoders::wristEncoder, address::practice_bot::encoders::wristEncoder, instance)} {
  /* ———————————————————————— MOTOR CONFIGURATION ———————————————————————— */

  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::shoulder,
                                         motorConfig::practice_bot::lifter::shoulder>(
      m_frontShoulder, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::shoulder,
                                         motorConfig::practice_bot::lifter::shoulder>(m_backShoulder, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::arm, motorConfig::practice_bot::lifter::arm>(
      m_arm, 100_ms, instance);

  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::lifter::wrist,
                                         motorConfig::practice_bot::lifter::wrist>(m_wrist, 100_ms, instance);

  // Make back shoulder motor follow front shoulder motor
  m_backShoulder.Follow(m_frontShoulder);
}

// This method will be called once per scheduler run
void lifter_subsystem::Periodic() {}
