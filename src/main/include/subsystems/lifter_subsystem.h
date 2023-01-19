/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <ctre/phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include <string>

/* —————————————————————————— SUBSYSTEM CLASS —————————————————————————— */

class LifterSubsystem : public frc2::SubsystemBase {
 public:
  explicit LifterSubsystem(argos_lib::RobotInstance instance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Shoulder motors are attached in parallel mechanically to operate shoulder, back motor follows front motor
  WPI_TalonFX m_shoulderLeader;    // Shoulder motor closest to front of robot
  WPI_TalonFX m_shoulderFollower;  // Shoulder motor closest to back of robot
  WPI_TalonFX m_arm;               // Motor that controls extension of arm
  WPI_TalonFX m_wrist;             // Motor that controls wrist movement
  CANCoder m_armEncoder;           // Encoder that measures arm extension
  CANCoder m_shoulderEncoder;      // Encoder that measures shoulder position
  CANCoder m_wristEncoder;         // Encoder for measuring wrist position
};
