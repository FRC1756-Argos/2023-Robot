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
    , m_coneLeftIntakeSensor(instance == argos_lib::RobotInstance::Competition ?
                                 address::comp_bot::sensors::tofConeIntakeLeft :
                                 address::practice_bot::sensors::tofConeIntakeLeft)
    , m_coneRightIntakeSensor(instance == argos_lib::RobotInstance::Competition ?
                                  address::comp_bot::sensors::tofConeIntakeRight :
                                  address::practice_bot::sensors::tofConeIntakeRight)
    , m_cubeIntakeSensor(instance == argos_lib::RobotInstance::Competition ?
                             address::comp_bot::sensors::tofCubeIntake :
                             address::practice_bot::sensors::tofCubeIntake)
    , m_haveCone(false)
    , m_haveCube(false)
    , m_coneOffsetFilter{frc::LinearFilter<units::inch_t>::SinglePoleIIR(0.2, 0.02_s)}
    , m_filteredConeOffset{0_in} {
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::intake,
                                             motorConfig::practice_bot::intake::intake>(
      m_intakeMotor, 100_ms, instance);
  m_coneLeftIntakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
  m_coneLeftIntakeSensor.SetRangeOfInterest(8, 8, 12, 12);
  m_coneRightIntakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
  m_coneRightIntakeSensor.SetRangeOfInterest(8, 8, 12, 12);
  m_cubeIntakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
  m_cubeIntakeSensor.SetRangeOfInterest(8, 8, 12, 12);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  auto rawConePosition = GetRawConePosition();
  frc::SmartDashboard::PutNumber("intake/raw cone position",
                                 rawConePosition ? rawConePosition.value().to<double>() : -1);

  m_filteredConeOffset = m_coneOffsetFilter.Calculate(rawConePosition ? rawConePosition.value() : m_filteredConeOffset);

  frc::SmartDashboard::PutNumber("intake/filtered cone position", m_filteredConeOffset.to<double>());

  frc::SmartDashboard::PutBoolean("intake/ConeDetection", IsConeDetected());
  frc::SmartDashboard::PutBoolean("intake/CubeDetection", IsCubeDetected());

  frc::SmartDashboard::PutNumber("intake/Stator Current", m_intakeMotor.GetStatorCurrent());
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

units::inch_t IntakeSubsystem::GetIntakeDistance() {
  return m_filteredConeOffset;
}

bool IntakeSubsystem::TofConeDetected() {
  return ReadSensorDistance(m_coneLeftIntakeSensor).has_value() &&
         ReadSensorDistance(m_coneRightIntakeSensor).has_value();
}

bool IntakeSubsystem::TofCubeDetected() {
  return ReadSensorDistance(m_cubeIntakeSensor).has_value();
}

bool IntakeSubsystem::IsConeDetected() {
  return TofConeDetected() && IsGamePieceDetected();
}

bool IntakeSubsystem::IsCubeDetected() {
  return TofCubeDetected() && IsGamePieceDetected();
}

bool IntakeSubsystem::IsGamePieceDetected() {
  return std::abs(m_intakeMotor.GetStatorCurrent()) > 7.0;
}

bool IntakeSubsystem::IsGamepieceLost() {
  return (m_haveCone || m_haveCube) && !IsGamePieceDetected();
}

std::optional<units::inch_t> IntakeSubsystem::ReadSensorDistance(frc::TimeOfFlight& sensor) {
  units::inch_t rawSensorDistance = units::make_unit<units::millimeter_t>(sensor.GetRange());
  if (rawSensorDistance > 16_in || rawSensorDistance < 0_in) {
    return std::nullopt;
  }
  return rawSensorDistance;
}

std::optional<units::inch_t> IntakeSubsystem::GetRawConePosition() {
  auto leftSensorDistance = ReadSensorDistance(m_coneLeftIntakeSensor);
  auto rightSensorDistance = ReadSensorDistance(m_coneRightIntakeSensor);

  frc::SmartDashboard::PutNumber("intake/raw left", leftSensorDistance ? leftSensorDistance.value().to<double>() : -1);
  frc::SmartDashboard::PutNumber("intake/raw right",
                                 rightSensorDistance ? rightSensorDistance.value().to<double>() : -1);

  auto sensorDistance = leftSensorDistance;
  // Sensors lose accuracy at very short distances, so if cone is closer to left sensor, use right sensor distance
  // but generate distance that left sensor should see
  if (rightSensorDistance && (!leftSensorDistance || rightSensorDistance.value() > leftSensorDistance.value())) {
    sensorDistance = measure_up::lifter::wrist::wristWidth - rightSensorDistance.value() - cone::coneWidth;
  }

  // * useful for wrist positions
  // if (m_robotInstance == argos_lib::RobotInstance::Competition) {
  //   sensorDistance = measure_up::lifter::wrist::wristWidth - sensorDistance;
  // }

  // * removed 1.5 inch fudge
  if (sensorDistance) {
    return (measure_up::lifter::wrist::wristWidth / 2) - (sensorDistance.value() + cone::coneWidth / 2);
  }

  return std::nullopt;
}
