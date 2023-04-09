/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/filter/LinearFilter.h>
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

  units::inch_t GetIntakeDistance();
  bool TofConeDetected();
  bool TofCubeDetected();
  bool IsConeDetected();  ///< For intake detection
  bool IsCubeDetected();  ///< For intake detection
  bool IsGamepieceLost();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonSRX m_intakeMotor;
  argos_lib::RobotInstance m_robotInstance;

  frc::TimeOfFlight
      m_coneLeftIntakeSensor;  ///< Time of flight distance sensor mounted on left side of intake (wheels up) to detect cone presence & location
  frc::TimeOfFlight
      m_coneRightIntakeSensor;  ///< Time of flight distance sensor mounted on right side of intake (wheels up) to detect cone presence & location
  frc::TimeOfFlight
      m_cubeIntakeSensor;  ///< Time of flight distance sensor mounted on left side of intake (wheels up) to detect cube presence & location
  bool m_haveCone;
  bool m_haveCube;

  frc::LinearFilter<units::inch_t> m_coneOffsetFilter;
  units::inch_t
      m_filteredConeOffset;  ///< Location of cone relative to intake center filtered to reduce noise.  Positive is toward left side (wheels up)

  bool IsGamePieceDetected();
  std::optional<units::inch_t> ReadSensorDistance(frc::TimeOfFlight& sensor);
  std::optional<units::inch_t> GetRawConePosition();
};
