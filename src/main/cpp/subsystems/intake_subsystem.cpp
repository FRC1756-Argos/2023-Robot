/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include "constants/addresses.h"

IntakeSubsystem::IntakeSubsystem(argos_lib::RobotInstance instance)
    : m_intakeMotor{instance == argos_lib::RobotInstance::Competition ?
                        address::comp_bot::intake::intakeMotor.address :
                        address::practice_bot::intake::intakeMotor.address} {}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}
