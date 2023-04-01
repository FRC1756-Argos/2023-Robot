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
    , m_haveCube(false) {
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::intake,
                                             motorConfig::practice_bot::intake::intake>(
      m_intakeMotor, 100_ms, instance);
  m_coneLeftIntakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
  m_coneLeftIntakeSensor.SetRangeOfInterest(8, 8, 12, 12);
  m_coneRightIntakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
  m_coneRightIntakeSensor.SetRangeOfInterest(8, 8, 12, 12);
  m_cubeIntakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
  m_cubeIntakeSensor.SetRangeOfInterest(8, 8, 12, 12);
  units::inch_t left_average[] = {units::make_unit<units::millimeter_t>(0.1),
                                  units::make_unit<units::millimeter_t>(0.1),
                                  units::make_unit<units::millimeter_t>(0.1),
                                  units::make_unit<units::millimeter_t>(0.1),
                                  units::make_unit<units::millimeter_t>(0.1)};

  units::inch_t right_average[] = {units::make_unit<units::millimeter_t>(0.1),
                                   units::make_unit<units::millimeter_t>(0.1),
                                   units::make_unit<units::millimeter_t>(0.1),
                                   units::make_unit<units::millimeter_t>(0.1),
                                   units::make_unit<units::millimeter_t>(0.1)};
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  std::optional<units::inch_t> sensDistance = GetIntakeDistance();
  frc::SmartDashboard::PutNumber("SensorDistance", sensDistance ? sensDistance.value().to<double>() : -1);

  units::inch_t leftSensorDistance = units::make_unit<units::millimeter_t>(m_coneLeftIntakeSensor.GetRange());
  frc::SmartDashboard::PutNumber("TOF Left", leftSensorDistance.value());
  frc::SmartDashboard::PutNumber("TOF Left Averaged", CalAverage(leftSensorDistance, left_average).value());

  units::inch_t rightSensorDistance = units::make_unit<units::millimeter_t>(m_coneRightIntakeSensor.GetRange());
  frc::SmartDashboard::PutNumber("TOF Right", rightSensorDistance.value());
  frc::SmartDashboard::PutNumber("TOF Right Averaged", CalAverage(rightSensorDistance, right_average).value());

  frc::SmartDashboard::PutNumber("Intake Stator Current", m_intakeMotor.GetStatorCurrent());

  frc::SmartDashboard::PutNumber(
      "TOF Cube", units::make_unit<units::millimeter_t>(m_cubeIntakeSensor.GetRange()).convert<units::inch>().value());
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
  if (!TofConeDetected()) {
    return std::nullopt;
  }

  units::inch_t leftSensorDistance = units::make_unit<units::millimeter_t>(m_coneLeftIntakeSensor.GetRange());
  leftSensorDistance = CalAverage(leftSensorDistance, left_average);
  units::inch_t rightSensorDistance = units::make_unit<units::millimeter_t>(m_coneRightIntakeSensor.GetRange());
  rightSensorDistance = CalAverage(rightSensorDistance, right_average);

  auto sensorDistance = leftSensorDistance;
  // Sensors lose accuracy at very short distances, so if cone is closer to left sensor, use right sensor distance
  // but generate distance that left sensor should see
  if (rightSensorDistance > leftSensorDistance) {
    sensorDistance = measure_up::lifter::wrist::wristWidth - rightSensorDistance - cone::coneWidth;
  }

  // * useful for wrist positions
  // if (m_robotInstance == argos_lib::RobotInstance::Competition) {
  //   sensorDistance = measure_up::lifter::wrist::wristWidth - sensorDistance;
  // }

  // * removed 1.5 inch fudge
  units::inch_t gamePieceDepth = (measure_up::lifter::wrist::wristWidth / 2) - (sensorDistance + cone::coneWidth / 2);

  return gamePieceDepth;
}

bool IntakeSubsystem::TofConeDetected() {
  return units::make_unit<units::millimeter_t>(m_coneLeftIntakeSensor.GetRange()) < 16_in;
}

bool IntakeSubsystem::TofCubeDetected() {
  return units::make_unit<units::millimeter_t>(m_cubeIntakeSensor.GetRange()) < 16_in;
}

bool IntakeSubsystem::IsConeDetected() {
  //disabled due to not working
  return TofConeDetected();  //&& IsGamePieceDetected();
}

bool IntakeSubsystem::IsCubeDetected() {
  return TofCubeDetected() && IsGamePieceDetected();
}

bool IntakeSubsystem::IsGamePieceDetected() {
  return std::abs(m_intakeMotor.GetStatorCurrent()) > 10.0;
}

units::inch_t IntakeSubsystem::CalAverage(units::inch_t new_val, units::inch_t average[5]) {
  units::inch_t sum = units::make_unit<units::inch_t>(0);
  for (int i = 0; i < 5; i++) {
    if (i + 1 < 5) {
      average[i] = average[i + 1];
    } else {
      average[i] = new_val;
    }
    sum += average[i];
  }

  if (sum == units::make_unit<units::inch_t>(0)) {
    return units::make_unit<units::inch_t>(0);
  }
  return sum / 5;
}
