/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/score_cone_command.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include <Constants.h>

#include "commands/set_arm_pose_command.h"

ScoreConeCommand::ScoreConeCommand(LifterSubsystem& lifter, BashGuardSubsystem& bash, IntakeSubsystem& intake)
    : m_lifter{lifter}
    , m_intake{intake}
    , m_bash{bash}
    , m_retractIntake{frc2::InstantCommand{[this]() { m_intake.EjectCone(); }, {&m_intake}}}
    , m_allCommands{frc2::InstantCommand{[]() {}, {}}} {}

// Called when the command is initially scheduled.
void ScoreConeCommand::Initialize() {
  auto gpScore = SetArmPoseCommand{m_lifter,
                                   m_bash,
                                   GetRelativePose(m_lifter.GetArmPose(WristPosition::Unknown), -10_in, -10_in),
                                   BashGuardPosition::Retracted,
                                   WristPosition::Unknown,
                                   PathType::concaveUp,
                                   speeds::armKinematicSpeeds::effectorVelocity,
                                   speeds::armKinematicSpeeds::effectorAcceleration};

  m_allCommands =
      frc2::ParallelCommandGroup(gpScore, frc2::SequentialCommandGroup(frc2::WaitCommand(500_ms), m_retractIntake))
          .ToPtr();
  // Initialize all commands
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void ScoreConeCommand::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void ScoreConeCommand::End(bool interrupted) {
  // When command is done, stop the intake reverseing
  m_allCommands.get()->End(interrupted);
  m_intake.IntakeStop();
}

// Returns true when the command should end.
bool ScoreConeCommand::IsFinished() {
  return m_allCommands.get()->IsFinished();
}
