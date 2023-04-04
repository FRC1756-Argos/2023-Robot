/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_drive_tuning.h"

#include <commands/drive_to_position.h>
#include <commands/grip_cone_command.h>
#include <commands/initialize_odometry_command.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

AutonomousDriveTuning::AutonomousDriveTuning(SwerveDriveSubsystem& drive,
                                             BashGuardSubsystem& bash,
                                             LifterSubsystem& lifter,
                                             SimpleLedSubsystem& leds,
                                             IntakeSubsystem& intake)
    : m_drive{drive}, m_bashGuard{bash}, m_lifter{lifter}, m_leds{leds}, m_intake{intake}, m_allCommands{nullptr} {
  frc::SmartDashboard::SetDefaultNumber("autoDriveTuning/initialAngle (deg)", 0);
  frc::SmartDashboard::SetDefaultNumber("autoDriveTuning/finalAngle (deg)", 0);
  frc::SmartDashboard::SetDefaultNumber("autoDriveTuning/deltaX (feet)", 10);
  frc::SmartDashboard::SetDefaultNumber("autoDriveTuning/deltaY (feet)", 0);
  frc::SmartDashboard::SetDefaultNumber("autoDriveTuning/targetVelocity (fps)", 10);
  frc::SmartDashboard::SetDefaultNumber("autoDriveTuning/targetAccel (fps^2)", 10);
  frc::SmartDashboard::SetDefaultNumber("autoDriveTuning/targetAngVelocity (deg_per_s)", 360);
  frc::SmartDashboard::SetDefaultNumber("autoDriveTuning/targetAngAccel (deg_per_s^2)", 360);
}

// Called when the command is initially scheduled.
void AutonomousDriveTuning::Initialize() {
  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);

  units::degree_t initialAngle{frc::SmartDashboard::GetNumber("autoDriveTuning/initialAngle (deg)", 0)};
  units::degree_t finalAngle{frc::SmartDashboard::GetNumber("autoDriveTuning/finalAngle (deg)", 0)};
  units::foot_t deltaX{frc::SmartDashboard::GetNumber("autoDriveTuning/deltaX (feet)", 10)};
  units::foot_t deltaY{frc::SmartDashboard::GetNumber("autoDriveTuning/deltaY (feet)", 0)};
  units::feet_per_second_t targetVelocity{frc::SmartDashboard::GetNumber("autoDriveTuning/targetVelocity (fps)", 10)};
  units::feet_per_second_squared_t targetAccel{
      frc::SmartDashboard::GetNumber("autoDriveTuning/targetAccel (fps^2)", 10)};
  units::degrees_per_second_t targetAngVelocity{
      frc::SmartDashboard::GetNumber("autoDriveTuning/targetAngVelocity (deg_per_s)", 360)};
  units::degrees_per_second_squared_t targetAngAccel{
      frc::SmartDashboard::GetNumber("autoDriveTuning/targetAngAccel (deg_per_s^2)", 360)};

  m_allCommands = InitializeOdometryCommand{&m_drive, {0_m, 0_m, initialAngle}}
                      .ToPtr()
                      .AndThen(DriveToPosition{
                          &m_drive,
                          {0_m, 0_m, initialAngle},
                          initialAngle,
                          {deltaX, deltaY, finalAngle},
                          finalAngle,
                          frc::TrapezoidProfile<units::inches>::Constraints{targetVelocity, targetAccel},
                          frc::TrapezoidProfile<units::degrees>::Constraints{targetAngVelocity, targetAngAccel}}
                                   .ToPtr())
                      .AndThen(DriveToPosition{
                          &m_drive,
                          {deltaX, deltaY, finalAngle},
                          finalAngle,
                          {0_m, 0_m, initialAngle},
                          initialAngle,
                          frc::TrapezoidProfile<units::inches>::Constraints{targetVelocity, targetAccel},
                          frc::TrapezoidProfile<units::degrees>::Constraints{targetAngVelocity, targetAngAccel}}
                                   .ToPtr());

  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousDriveTuning::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutonomousDriveTuning::End(bool interrupted) {
  m_allCommands.get()->End(interrupted);
}

// Returns true when the command should end.
bool AutonomousDriveTuning::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

std::string AutonomousDriveTuning::GetName() const {
  return "Drive Tuning";
}

frc2::Command* AutonomousDriveTuning::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
