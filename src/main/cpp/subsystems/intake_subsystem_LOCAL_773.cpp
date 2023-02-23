/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/config/talonsrx_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"
#include "subsystems/intake_subsystem.h"

IntakeSubsystem::IntakeSubsystem(argos_lib::RobotInstance instance)
    : m_intakeMotor{GetCANAddr(
          address::comp_bot::intake::intakeMotor, address::comp_bot::intake::intakeMotor, instance)}
    , m_intakeSensor(instance == argos_lib::RobotInstance::Competition ?
                         address::comp_bot::sensors::tofSensorIntake :
                         address::practice_bot::sensors::tofSensorIntake) {
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::intake,
                                             motorConfig::practice_bot::intake::intake>(
      m_intakeMotor, 100_ms, instance);
  m_intakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
  m_intakeSensor.SetRangeOfInterest(8, 8, 12, 12);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  std::printf("Sensor Distance, %f\n", GetIntakeDistance().to<double>());
}

void IntakeSubsystem::IntakeForward() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 1.0);
}
void IntakeSubsystem::IntakeReverse() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.2);
}
void IntakeSubsystem::IntakeFastReverse() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.6);
}

void IntakeSubsystem::IntakeStop() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0.0);
}

void IntakeSubsystem::Disable() {
  IntakeStop();
}

units::inch_t IntakeSubsystem::GetIntakeDistance() {
  units::inch_t GetIntakeDistance = units::make_unit<units::millimeter_t>(m_intakeSensor.GetRange());
  return GetIntakeDistance;
}
