/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/falcon_config.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

class BashGuardSubsystem : public frc2::SubsystemBase {
 public:
  explicit BashGuardSubsystem(argos_lib::RobotInstance instance);

  void SetBashGuardManualOverride(bool overrideState);

  /// @brief Sets bash guard extension speed
  /// @param speed double, on the interval [-1, 1]
  void SetExtensionSpeed(double speed);

  void SetExtensionLength(units::inch_t length);

  bool IsBashGuardHomed();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void UpdateExtensionHome();

  bool IsBashGuardManualOverride();

  bool IsBashGuardMoving();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_bashGuard;
  bool m_bashGuardManualOverride;
  bool m_bashGuardHomed;
};
