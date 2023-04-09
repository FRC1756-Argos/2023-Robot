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

  /// @brief Sets the speed of placer mechanism motor
  /// @param percentOutput Fration of max power to set motor to, bounded on [-1, 1]
  void SetOuiOuiSpeed(double percentOutput);

  /// @brief Stops the oui oui placer, and sets to motor's neutural mode
  void StopOuiOuiPlacer();

  /// @brief Retrieves the current angle the oui oui placer
  /// @return Angle of oui oui placer system relative to robot frame, specifically it's revolution around robot Y
  units::degree_t GetOuiOuiAngle();

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
