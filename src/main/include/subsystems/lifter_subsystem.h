// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <argos_lib/config/config_types.h>

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::sensors;

/* —————————————————————————— SUBSYSTEM CLASS —————————————————————————— */

class lifter_subsystem : public frc2::SubsystemBase {
 public:
  lifter_subsystem(argos_lib::RobotInstance instance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Gets the CAN numerical address from a CANAddress object given a robot instance
  static inline int GetCANAddr(const argos_lib::CANAddress& compAddress,
                               const argos_lib::CANAddress& practiceAddress,
                               argos_lib::RobotInstance instance) {
    if (instance == argos_lib::RobotInstance::Competition) {
      return compAddress.address;
    } else {
      return practiceAddress.address;
    }
  }

  // Gets the CAN bus name from a CANAddress given a robot instance
  static inline std::string GetCANBus(const argos_lib::CANAddress& compAddress,
                                      const argos_lib::CANAddress& practiceAddress,
                                      argos_lib::RobotInstance instance) {
    if (instance == argos_lib::RobotInstance::Competition) {
      return std::string(compAddress.busName);
    } else {
      return std::string(practiceAddress.busName);
    }
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
