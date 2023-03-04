/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include <TimeOfFlight.h>

#include "argos_lib/config/config_types.h"
#include "argos_lib/general/hysteresis_filter.h"
#include "units/length.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  explicit IntakeSubsystem(argos_lib::RobotInstance instance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void IntakeForward();
  void IntakeReverse();
  void IntakeFastReverse();

  void IntakeCone();
  void EjectCone();
  void EjectConeForReal();
  void IntakeCube();
  void EjectCube();
  void IntakeStop();
  void Disable();

  /// @brief ToF stuff
  /// @return
  std::optional<units::inch_t> GetIntakeDistance();
  bool IsConeDetected();
  bool IsCubeDetected();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonSRX m_intakeMotor;
  argos_lib::RobotInstance m_robotInstance;

  /// @brief ToF sensor thingy
  frc::TimeOfFlight m_intakeSensor1;
  frc::TimeOfFlight m_intakeSensor2;
  bool m_haveCone;
  bool m_haveCube;
};
