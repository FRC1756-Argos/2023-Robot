/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_balance.h"

#include <commands/drive_to_position.h>
#include <commands/initialize_odometry_command.h>
#include <commands/set_arm_pose_command.h>
#include <frc2/command/WaitCommand.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "commands/balance_charging_station.h"
#include "commands/drive_over_charging_station.h"
#include "commands/place_cone_command.h"
#include "constants/scoring_positions.h"

AutonomousBalance::AutonomousBalance(SwerveDriveSubsystem& drive,
                                     BashGuardSubsystem& bash,
                                     LifterSubsystem& lifter,
                                     SimpleLedSubsystem& leds,
                                     IntakeSubsystem& intake)
    : m_drive{drive}
    , m_bashGuard{bash}
    , m_lifter{lifter}
    , m_leds{leds}
    , m_intake{intake}
    , m_allCommands{
          (InitializeOdometryCommand{&m_drive, {0_m, 0_m, 180_deg}}.ToPtr())
              .AndThen(PlaceConeCommand{
                  &bash,
                  &lifter,
                  &intake,
                  scoring_positions::lifter_extension_end::coneHigh.lifterPosition,
                  ScoringPosition{.column = ScoringColumn::leftGrid_leftCone, .row = ScoringRow::high},
              }
                           .ToPtr())
              .AndThen(SetArmPoseCommand{&lifter,
                                         &bash,
                                         ScoringPosition{.column = ScoringColumn::stow},
                                         []() { return false; },
                                         []() { return false; },
                                         PathType::concaveDown}
                           .ToPtr()
                           .AlongWith(DriveOverChargingStation{&m_drive, 0_deg, 180_deg}.ToPtr()))
              .AndThen(frc2::WaitCommand(500_ms).ToPtr())
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
  return "Mobility Balance";
}

frc2::Command* AutonomousBalance::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
