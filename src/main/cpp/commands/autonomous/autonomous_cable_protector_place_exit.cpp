/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_cable_protector_place_exit.h"

#include <commands/drive_to_position.h>
#include <commands/grip_cone_command.h>
#include <commands/initialize_odometry_command.h>
#include <commands/place_cone_command.h>
#include <constants/field_points.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

AutonomousCableProtectorPlaceExit::AutonomousCableProtectorPlaceExit(SwerveDriveSubsystem& drive,
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
                                                   frc::Translation2d{0_in, 0_in},
                                                   []() { return false; },
                                                   []() { return false; },
                                                   PathType::concaveDown}
                                     .ToPtr()
                                     .AlongWith(DriveToPosition{&m_drive,
                                                                {0_m, 0_m, 180_deg},
                                                                180_deg,
                                                                {12_ft + 6_in, 0_m, 180_deg},
                                                                180_deg,
                                                                frc::TrapezoidProfile<units::inches>::Constraints{
                                                                    5_fps, units::feet_per_second_squared_t{12}},
                                                                frc::TrapezoidProfile<units::degrees>::Constraints{
                                                                    units::degrees_per_second_t{360},
                                                                    units::degrees_per_second_squared_t{360}}}
                                                    .ToPtr()))} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutonomousCableProtectorPlaceExit::Initialize() {
  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCableProtectorPlaceExit::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCableProtectorPlaceExit::End(bool interrupted) {
  m_allCommands.get()->End(interrupted);
}

// Returns true when the command should end.
bool AutonomousCableProtectorPlaceExit::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

std::string AutonomousCableProtectorPlaceExit::GetName() const {
  return "Cable Protector Place & Exit";
}

frc2::Command* AutonomousCableProtectorPlaceExit::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
