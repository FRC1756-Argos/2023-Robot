/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/place_cone_command.h"

#include <commands/drive_to_position.h>
#include <commands/initialize_odometry_command.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "commands/grip_cone_command.h"
#include "commands/score_cone_command.h"
#include "commands/set_arm_pose_command.h"
#include "constants/scoring_positions.h"

PlaceConeCommand::PlaceConeCommand(BashGuardSubsystem* bash,
                                   LifterSubsystem* lifter,
                                   IntakeSubsystem* intake,
                                   frc::Translation2d m_desiredArmPos,
                                   ScoringPosition scoringPos)
    : m_bashGuard{bash}
    , m_lifter{lifter}
    , m_intake{intake}
    , m_desiredArmPos{m_desiredArmPos}
    , m_scoringPosition{scoringPos}
    , m_allCommands{frc2::InstantCommand{[]() {}}} {}

// Called when the command is initially scheduled.
void PlaceConeCommand::Initialize() {
  m_allCommands =
      GripConeCommand(m_intake)
          .ToPtr()
          .AlongWith(frc2::InstantCommand([this]() {
                       LifterSubsystem& lifterRef = *m_lifter;
                       m_lifter->SetShoulderAngle(PlaceConeCommand::GetShoulderAngle(lifterRef, m_desiredArmPos));
                     }).ToPtr())
          .AndThen(frc2::WaitUntilCommand([this]() {
                     return m_bashGuard->IsBashGuardHomed() && m_lifter->IsArmExtensionHomed();
                   }).ToPtr())
          .AndThen(SetArmPoseCommand{m_lifter,
                                     m_bashGuard,
                                     m_scoringPosition,
                                     frc::Translation2d{0_in, 0_in},
                                     []() { return true; },
                                     []() { return false; },
                                     PathType::concaveDown}
                       .ToPtr())
          .AndThen(ScoreConeCommand{*m_lifter, *m_bashGuard, *m_intake}.ToPtr());
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
  bool isFinished = m_allCommands.get()->IsFinished();
  frc::SmartDashboard::PutBoolean("Place Cone Command Is Finished: ", isFinished);
  return isFinished;
}

std::string PlaceConeCommand::GetName() const {
  return "Place Cone";
}

frc2::Command* PlaceConeCommand::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
