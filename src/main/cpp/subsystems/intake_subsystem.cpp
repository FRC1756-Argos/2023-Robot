/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "argos_lib/config/talonsrx_config.h"
#include "constants/addresses.h"
#include "constants/field_points.h"
#include "constants/measure_up.h"
#include "constants/motors.h"

IntakeSubsystem::IntakeSubsystem(argos_lib::RobotInstance instance)
    : m_intakeMotor{GetCANAddr(
          address::comp_bot::intake::intakeMotor, address::comp_bot::intake::intakeMotor, instance)}
    , m_robotInstance{instance}
    , m_coneIntakeSensor(instance == argos_lib::RobotInstance::Competition ?
                             address::comp_bot::sensors::tofConeIntake :
                             address::practice_bot::sensors::tofConeIntake)
    , m_cubeIntakeSensor(instance == argos_lib::RobotInstance::Competition ?
                             address::comp_bot::sensors::tofCubeIntake :
                             address::practice_bot::sensors::tofCubeIntake)
    , m_haveCone(false)
    , m_haveCube(false) {
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::intake,
                                             motorConfig::practice_bot::intake::intake>(
      m_intakeMotor, 100_ms, instance);
  m_coneIntakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
  m_coneIntakeSensor.SetRangeOfInterest(8, 8, 12, 12);
  m_cubeIntakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
  m_cubeIntakeSensor.SetRangeOfInterest(8, 8, 12, 12);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  std::optional<units::inch_t> sensDistance = GetIntakeDistance();
  frc::SmartDashboard::PutNumber("SensorDistance", sensDistance ? sensDistance.value().to<double>() : -1);
  frc::SmartDashboard::PutBoolean("ConeDetection", IsConeDetected());
  frc::SmartDashboard::PutBoolean("CubeDetection", IsCubeDetected());
}

void IntakeSubsystem::IntakeCone() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 1.0);
  m_haveCone = true;
  m_haveCube = false;
}
void IntakeSubsystem::EjectCone() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.2);
  m_haveCone = false;
}
void IntakeSubsystem::EjectConeForReal() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -1.0);
  m_haveCone = false;
}
void IntakeSubsystem::IntakeCube() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.7);
  m_haveCube = true;
  m_haveCone = false;
}
void IntakeSubsystem::EjectCube() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0.7);
  m_haveCube = false;
}

void IntakeSubsystem::IntakeStop() {
  if (m_haveCone) {
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0.3);
  } else if (m_haveCube) {
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.6);
  } else {
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0.0);
  }
}

void IntakeSubsystem::Disable() {
  m_haveCone = false;
  m_haveCube = false;
  IntakeStop();
}

std::optional<units::inch_t> IntakeSubsystem::GetIntakeDistance() {
  if (!IsConeDetected()) {
    return std::nullopt;
  }

  units::inch_t sensorDistance = units::make_unit<units::millimeter_t>(m_coneIntakeSensor.GetRange());

  // * useful for wrist positions
  // if (m_robotInstance == argos_lib::RobotInstance::Competition) {
  //   sensorDistance = measure_up::lifter::wrist::wristWidth - sensorDistance;
  // }

  // * removed 1.5 inch fudge
  units::inch_t gamePieceDepth = (measure_up::lifter::wrist::wristWidth / 2) - (sensorDistance + cone::coneWidth / 2);

  return gamePieceDepth;
}

bool IntakeSubsystem::IsConeDetected() {
  units::inch_t GetIntakeDistance = units::make_unit<units::millimeter_t>(m_coneIntakeSensor.GetRange());
  if (GetIntakeDistance < 16_in) {
    return true;
  }
  return false;
}

bool IntakeSubsystem::IsCubeDetected() {
  units::inch_t GetIntakeDistance = units::make_unit<units::millimeter_t>(m_cubeIntakeSensor.GetRange());
  if (GetIntakeDistance < 16_in) {
    return true;
  }
  return false;
}
