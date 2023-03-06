/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/place_cone_command.h"

#include <commands/drive_to_position.h>
#include <commands/initialize_odometry_command.h>
#include <frc2/command/WaitCommand.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "commands/grip_cone_command.h"
#include "commands/score_cone_command.h"
#include "commands/set_arm_pose_command.h"
#include "constants/scoring_positions.h"

PlaceConeCommand::PlaceConeCommand(BashGuardSubsystem& bash,
                                   LifterSubsystem& lifter,
                                   SimpleLedSubsystem& leds,
                                   IntakeSubsystem& intake,
                                   frc::Translation2d desArmPos,
                                   ScoringPosition scoringPos)
    : m_bashGuard{bash}
    , m_lifter{lifter}
    , m_leds{leds}
    , m_intake{intake}
    , m_desiredArmPos{desArmPos}
    , m_scoringPosition{scoringPos}
    , m_allCommands{
          ((GripConeCommand(m_lifter, m_bashGuard, m_intake))
               .ToPtr()
               .AlongWith(
                   frc2::InstantCommand(
                       [this]() {
                         m_lifter.SetShoulderAngle(this->GetShoulderAngle(
                             m_lifter,
                             m_desiredArmPos));  // scoring_positions::lifter_extension_end::coneHigh.lifterPosition
                       },
                       {&m_lifter})
                       .ToPtr()))
              .AndThen(frc2::WaitCommand(1_s).ToPtr())
              .AndThen(
                  SetArmPoseCommand(
                      m_lifter,
                      m_bashGuard,
                      m_scoringPosition,  // ScoringPosition{.column = ScoringColumn::leftGrid_leftCone, .row = ScoringRow::high},
                      [this]() { return false; },
                      [this]() { return false; },
                      PathType::unmodified)
                      .ToPtr())
              .AndThen(ScoreConeCommand{m_lifter, m_bashGuard, m_intake}.ToPtr())
              .AndThen(SetArmPoseCommand(
                           m_lifter,
                           m_bashGuard,
                           ScoringPosition{.column = ScoringColumn::stow},
                           [this]() { return false; },
                           [this]() { return false; },
                           PathType::concaveDown)
                           .ToPtr()),
      } {}

// Called when the command is initially scheduled.
void PlaceConeCommand::Initialize() {
  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void PlaceConeCommand::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void PlaceConeCommand::End(bool interrupted) {
  m_allCommands.get()->End(interrupted);
}

// Returns true when the command should end.
bool PlaceConeCommand::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

std::string PlaceConeCommand::GetName() const {
  return "Place Cone";
}

frc2::Command* PlaceConeCommand::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
