/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <argos_lib/general/nt_motor_pid_tuner.h>
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

class OuiOuiPlacerSubsystem : public frc2::SubsystemBase {
 public:
  explicit OuiOuiPlacerSubsystem(argos_lib::RobotInstance instance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /// @brief Sets manual override to true, disabling closed loop control of any kind
  void TakeManualControl();

  /// @brief Sets manual override to false, giving control back to closed loop
  void ReleaseManualControl();

  /// @brief Gets the manual override status of subsystem
  /// @return True -> Manual override engaged. False -> Manual override NOT engaged.
  bool ReadManualControl();

  /// @brief Sets the speed of placer mechanism motor
  /// @param percentOutput Fration of max power to set motor to, bounded on [-1, 1]
  void SetOuiOuiSpeed(double percentOutput);

  // * NOTE: Angle follows WPI standard, decreasing angle will result in moving the mechanism down out
  // * the back of the robot, increasing will result in placer moving up back into the A-Frame
  /// @brief Sets the desired angle of the placer, relative to robot 0° pitch
  /// @param angle The angle to set placer to, angle has respect to robot y and follows WPI convention
  void SetOuiOuiAngle(units::degree_t angle);

  /// @brief Turns on soft limits of placer
  void EnablePlacerSoftLimits();

  /// @brief Turns off soft limits of placer
  void DisablePlacerSoftLimits();

 private:
  argos_lib::RobotInstance m_instance;
  bool m_manualOverride;
  WPI_TalonFX m_ouiOuiDrive;
  argos_lib::NTMotorPIDTuner m_ouiOuiTuner;
};
