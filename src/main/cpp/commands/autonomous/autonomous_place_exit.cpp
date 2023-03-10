/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_place_exit.h"

#include <commands/drive_to_position.h>
#include <commands/grip_cone_command.h>
#include <commands/initialize_odometry_command.h>
#include <commands/place_cone_command.h>
#include <constants/field_points.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

AutonomousPlaceExit::AutonomousPlaceExit(SwerveDriveSubsystem& drive,
                                         BashGuardSubsystem& bash,
                                         LifterSubsystem& lifter,
                                         SimpleLedSubsystem& leds,
                                         IntakeSubsystem& intake)
    : m_drive{drive}
    , m_bashGuard{bash}
    , m_lifter{lifter}
    , m_leds{leds}
    , m_intake{intake}
    , m_allCommands{(InitializeOdometryCommand{&m_drive, {0_m, 0_m, 180_deg}}.ToPtr())
                        .AndThen(PlaceConeCommand{&m_bashGuard,
                                                  &m_lifter,
                                                  &m_intake,
                                                  scoring_positions::lifter_extension_end::coneHigh.lifterPosition,
                                                  ScoringPosition(ScoringColumn::leftGrid_leftCone, ScoringRow::high)}
                                     .ToPtr())
                        .AndThen(SetArmPoseCommand{&lifter,
                                                   &bash,
                                                   ScoringPosition{.column = ScoringColumn::stow},
                                                   []() { return false; },
                                                   []() { return false; },
                                                   PathType::concaveDown}
                                     .ToPtr()
                                     .AlongWith(DriveToPosition{&m_drive,
                                                                {0_m, 0_m, 180_deg},
                                                                180_deg,
                                                                {-2_m, 0_m, 180_deg},
                                                                180_deg,
                                                                frc::TrapezoidProfile<units::inches>::Constraints{
                                                                    10_fps, units::feet_per_second_squared_t{12}},
                                                                frc::TrapezoidProfile<units::degrees>::Constraints{
                                                                    units::degrees_per_second_t{360},
                                                                    units::degrees_per_second_squared_t{360}}}
                                                    .ToPtr()))} {}

// Called when the command is initially scheduled.
void AutonomousPlaceExit::Initialize() {
  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousPlaceExit::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutonomousPlaceExit::End(bool interrupted) {
  m_allCommands.get()->End(interrupted);
}

// Returns true when the command should end.
bool AutonomousPlaceExit::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

std::string AutonomousPlaceExit::GetName() const {
  return "Place and Exit";
}

frc2::Command* AutonomousPlaceExit::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
