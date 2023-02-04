/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <ctre/Phoenix.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/SubsystemBase.h>

#include <string>

#include "argos_lib/homing/fs_homing.h"
#include "constants/interpolation_maps.h"
#include "utils/lifter_kinematics.h"

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

  /// @brief Sets the arm motor to zero
  void StopArm();

  /// @brief Sets the arm extension motor to zero
  void StopArmExtension();

  /// @brief Sets the wrist speed to zero
  void StopWrist();

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

  /// @brief Initializes the homed shoulder value from FS
  void InitializeShoulderHome();

  /// @brief Updates shoulder home in FS, resets relative position on sensor
  void UpdateShoulderHome();

  void SetShoulderAngle(units::degree_t angle);

  /// @brief Gets the shoulder's current angle relative to the arm's frame
  /// @return A units::degree_t representing the angle from zero
  units::degree_t GetShoulderAngle();

  /// @brief Retrieves the arm length
  /// @return A units::inch_t representing the length of the arm (from rotation center to end of arm)
  units::inch_t GetArmLen();

  /// @brief Uses current lifter state to give a pose of end effector
  /// @param state
  /// @return A Translation2d representing the position of the middle of the effector
  frc::Translation2d GetEffectorPos(LifterState state);

  /// @brief Gets the point on the very tip of the arm, without any effector offset
  /// @param state The current state of the arm system (len and angle)
  /// @return The point sitting on the end of the arm, in the middle
  frc::Translation2d GetArmEndPos(LifterState state);

  LifterState GetLifterState(frc::Translation2d desiredPose, bool effector);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Shoulder motors are attached in parallel mechanically to operate shoulder, back motor follows front motor
  WPI_TalonFX m_shoulderLeader;    ///< Shoulder motor closest to front of robot
  WPI_TalonFX m_shoulderFollower;  ///< Shoulder motor closest to back of robot
  WPI_TalonFX m_armExtension;      ///< Motor that controls extension of arm
  WPI_TalonFX m_wrist;             ///< Motor that controls wrist movement
  CANCoder m_armExtensionEncoder;  ///< Encoder that measures arm extension
  CANCoder m_shoulderEncoder;      ///< Encoder that measures shoulder position
  CANCoder m_wristEncoder;         ///< Encoder for measuring wrist position
  argos_lib::FSHomingStorage<units::degree_t> m_wristHomingStorage;
  bool m_wristHomed;
  argos_lib::FSHomingStorage<units::degree_t> m_shoulderHomeStorage;
  bool m_shoulderHomed;
};
