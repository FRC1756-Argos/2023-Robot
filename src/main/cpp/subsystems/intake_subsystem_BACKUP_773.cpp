/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include "argos_lib/config/talonsrx_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"

IntakeSubsystem::IntakeSubsystem(argos_lib::RobotInstance instance)
    : m_intakeMotor{GetCANAddr(
<<<<<<< HEAD
          address::comp_bot::intake::intakeMotor, address::comp_bot::intake::intakeMotor, instance)}
    , m_intakeSensor(instance == argos_lib::RobotInstance::Competition ?
                         address::comp_bot::sensors::tofSensorIntake :
                         address::practice_bot::sensors::tofSensorIntake) {
=======
          address::comp_bot::intake::intakeMotor, address::practice_bot::intake::intakeMotor, instance)}
    , m_haveCone(false)
    , m_haveCube(false) {
>>>>>>> main
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

void IntakeSubsystem::IntakeCone() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 1.0);
  m_haveCone = true;
  m_haveCube = false;
}
void IntakeSubsystem::EjectCone() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.2);
  m_haveCone = false;
}
void IntakeSubsystem::IntakeCube() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.7);
  m_haveCube = true;
  m_haveCone = false;
}
void IntakeSubsystem::EjectCube() {
  m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.7);
  m_haveCube = false;
}

void IntakeSubsystem::IntakeStop() {
  if (m_haveCone) {
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0.2);
  } else if (m_haveCube) {
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, -0.2);
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
  units::inch_t GetIntakeDistance = units::make_unit<units::millimeter_t>(m_intakeSensor.GetRange());
  return GetIntakeDistance;
}
