/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include <string>

#include "argos_lib/homing/fs_homing.h"
#include "constants/interpolation_maps.h"

/* —————————————————————————— SUBSYSTEM CLASS —————————————————————————— */

class LifterSubsystem : public frc2::SubsystemBase {
 public:
  explicit LifterSubsystem(argos_lib::RobotInstance instance);

  /// @brief Sets the arm speed
  /// @param speed double, on the interval [-1, 1]
  void SetShoulderSpeed(double speed);

  /// @brief Sets the arm extension motor speed
  /// @param speed double, on the interval [-1, 1]
  void SetArmExtensionSpeed(double speed);

  /// @brief Sets the wrist speed
  /// @param speed double, on the interval [-1, 1]
  void SetWristSpeed(double speed);

  /// @brief Sets the shoulder motor to zero
  void StopShoulder();

  /// @brief Sets the arm extension motor to zero
  void StopArmExtension();

  /// @brief Sets the wrist speed to zero
  void StopWrist();

  bool IsManualOverride();

  /// @brief initializing wrist homes from
  void InitializeWristHomes();

  /// @brief updating wrist homes for encoder
  void UpdateWristHome();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /// @brief Handle robot disabling
  void Disable();

  /// @brief Detect if arm is in motion
  /// @return True when arm is in motion
  bool IsArmMoving();

  /// @brief Update arm home position
  void UpdateArmHome();

  /// @brief Initializes the homed shoulder value from FS
  void InitializeShoulderHome();

  /// @brief Updates shoulder home in FS, resets relative position on sensor
  void UpdateShoulderHome();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Shoulder motors are attached in parallel mechanically to operate shoulder, back motor follows front motor
  WPI_TalonFX m_shoulderLeader;     ///< Shoulder motor closest to front of robot
  WPI_TalonFX m_shoulderFollower;   ///< Shoulder motor closest to back of robot
  WPI_TalonFX m_armExtensionMotor;  ///< Motor that controls extension of arm
  WPI_TalonFX m_wrist;              ///< Motor that controls wrist movement
  CANCoder m_armExtensionEncoder;   ///< Encoder that measures arm extension
  CANCoder m_shoulderEncoder;       ///< Encoder that measures shoulder position
  CANCoder m_wristEncoder;          ///< Encoder for measuring wrist position
  argos_lib::FSHomingStorage<units::degree_t> m_wristHomingStorage;
  bool m_wristHomed;
  argos_lib::FSHomingStorage<units::degree_t> m_shoulderHomeStorage;
  bool m_shoulderHomed;
};
