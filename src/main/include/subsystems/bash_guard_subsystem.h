/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/falcon_config.h>
#include <frc2/command/SubsystemBase.h>

class BashGuardSubsystem : public frc2::SubsystemBase {
 public:
  explicit BashGuardSubsystem(argos_lib::RobotInstance instance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_bashGaurd;
};
