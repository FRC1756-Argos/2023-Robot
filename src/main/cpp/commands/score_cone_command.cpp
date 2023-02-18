/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/score_cone_command.h"

#include <Constants.h>

#include "commands/set_arm_pose_command.h"

ScoreConeCommand::ScoreConeCommand(LifterSubsystem& lifter, BashGuardSubsystem& bash, IntakeSubsystem& intake)
    : m_lifter{lifter}
    , m_intake{intake}
    , m_bash{bash}
    , m_retractIntake{frc2::InstantCommand{[this]() { m_intake.IntakeFastReverse(); }, {&m_intake}}} {}

// Called when the command is initially scheduled.
void ScoreConeCommand::Initialize() {
  auto gpDown = SetArmPoseCommand{m_lifter,
                                  m_bash,
                                  GetRelativePose(m_lifter.GetArmPose(), 0_in, -10_in),
                                  BashGuardPosition::Retracted,
                                  speeds::armKinematicSpeeds::effectorVelocity,
                                  speeds::armKinematicSpeeds::effectorAcceleration,
                                  false};
  auto gpBack = SetArmPoseCommand{m_lifter,
                                  m_bash,
                                  GetRelativePose(m_lifter.GetArmPose(), -10_in, -10_in),
                                  BashGuardPosition::Retracted,
                                  speeds::armKinematicSpeeds::effectorVelocity,
                                  speeds::armKinematicSpeeds::effectorAcceleration,
                                  false};

  m_allCommands =
      std::make_unique<frc2::SequentialCommandGroup>(frc2::SequentialCommandGroup(gpDown, m_retractIntake, gpBack));
  // Initialize all commands
  m_allCommands->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void ScoreConeCommand::Execute() {
  m_allCommands->Execute();
}

// Called once the command ends or is interrupted.
void ScoreConeCommand::End(bool interrupted) {
  // When command is done, stop the intake reverseing
  m_allCommands->End(interrupted);
  m_intake.IntakeStop();
}

// Returns true when the command should end.
bool ScoreConeCommand::IsFinished() {
  return m_allCommands->IsFinished();
}
