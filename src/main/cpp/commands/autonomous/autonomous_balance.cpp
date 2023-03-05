/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_balance.h"

#include <commands/drive_to_position.h>
#include <commands/initialize_odometry_command.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "commands/balance_charging_station.h"
#include "commands/drive_over_charging_station.h"

AutonomousBalance::AutonomousBalance(SwerveDriveSubsystem& drive,
                                     BashGuardSubsystem& bash,
                                     LifterSubsystem& lifter,
                                     SimpleLedSubsystem& leds)
    : m_drive{drive}
    , m_bashGuard{bash}
    , m_lifter{lifter}
    , m_leds{leds}
    , m_allCommands{
          (InitializeOdometryCommand{&m_drive, {0_m, 0_m, 180_deg}}.ToPtr())
              .AndThen(DriveOverChargingStation{&m_drive, 0_deg, 180_deg}.ToPtr())
              .AndThen(BalanceChargingStation{&m_drive, 180_deg, 180_deg}.ToPtr()),
      } {}

// Called when the command is initially scheduled.
void AutonomousBalance::Initialize() {
  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousBalance::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutonomousBalance::End(bool interrupted) {
  if (interrupted) {
    m_allCommands.Cancel();
  }
}

// Returns true when the command should end.
bool AutonomousBalance::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

std::string AutonomousBalance::GetName() const {
  return "Balance";
}

frc2::Command* AutonomousBalance::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
