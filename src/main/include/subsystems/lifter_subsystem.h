/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <ctre/phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include <string>

/* —————————————————————————— SUBSYSTEM CLASS —————————————————————————— */

class lifter_subsystem : public frc2::SubsystemBase {
 public:
  explicit lifter_subsystem(argos_lib::RobotInstance instance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Gets the CAN numerical address from a CANAddress object given a robot instance
  static inline int GetCANAddr(const argos_lib::CANAddress& compAddress,
                               const argos_lib::CANAddress& practiceAddress,
                               argos_lib::RobotInstance instance) {
    return instance == argos_lib::RobotInstance::Competition ? compAddress.address : practiceAddress.address;
  }

  // Gets the CAN bus name from a CANAddress given a robot instance
  static inline std::string GetCANBus(const argos_lib::CANAddress& compAddress,
                                      const argos_lib::CANAddress& practiceAddress,
                                      argos_lib::RobotInstance instance) {
    return instance == argos_lib::RobotInstance::Competition ? std::string(compAddress.busName) :
                                                               std::string(practiceAddress.busName);
  }

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Shoulder motors are attached in parallel mechanically to operate shoulder, back motor follows front motor
  WPI_TalonFX m_frontShoulder;  // Shoulder motor closest to front of robot
  WPI_TalonFX m_backShoulder;   // Shoulder motor closest to back of robot
  WPI_TalonFX m_arm;            // Motor that controls extension of arm
  WPI_TalonFX m_wrist;          // Motor that controls wrist movement
  CANCoder m_armEncoder;        // Encoder that measures arm extension
  CANCoder m_shoulderEncoder;   // Encoder that measures shoulder position
  CANCoder m_wristEncoder;      // Encoder for measuring wrist position
};
