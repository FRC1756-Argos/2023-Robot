/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_drive_forward.h"

#include <commands/drive_to_position.h>
#include <commands/initialize_odometry_command.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

AutonomousDriveForward::AutonomousDriveForward(SwerveDriveSubsystem& drive,
                                               BashGuardSubsystem& bash,
                                               LifterSubsystem& lifter,
                                               SimpleLedSubsystem& leds)
    : m_drive{drive}
    , m_bashGuard{bash}
    , m_lifter{lifter}
    , m_leds{leds}
    , m_allCommands{InitializeOdometryCommand{&m_drive, {0_m, 0_m, 0_deg}},
                    DriveToPosition{
                        &m_drive,
                        {0_m, 0_m, 0_deg},
                        0_deg,
                        {2_m, 0_m, 0_deg},
                        0_deg,
                        frc::TrapezoidProfile<units::inches>::Constraints{10_fps, units::feet_per_second_squared_t{12}},
                        frc::TrapezoidProfile<units::degrees>::Constraints{
                            units::degrees_per_second_t{360}, units::degrees_per_second_squared_t{360}}}} {}

// Called when the command is initially scheduled.
void AutonomousDriveForward::Initialize() {
  m_leds.SetAllGroupsFlash(m_leds.GetAllianceColor());
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousDriveForward::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousDriveForward::End(bool interrupted) {
  m_allCommands.End(interrupted);
  m_leds.SetAllGroupsAllianceColor(false);
}

// Returns true when the command should end.
bool AutonomousDriveForward::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousDriveForward::GetName() const {
  return "Drive Forward";
}

frc2::Command* AutonomousDriveForward::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
