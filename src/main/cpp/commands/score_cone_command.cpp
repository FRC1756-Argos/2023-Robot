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
  m_lifter.ResetPathFaults();
  auto gpScore = SetArmPoseCommand{m_lifter,
                                   m_bash,
                                   GetRelativePose(m_lifter.GetArmPose(), -6_in, -8_in),
                                   BashGuardPosition::Retracted,
                                   WristPosition::Unknown,
                                   PathType::concaveUp,
                                   30_ips,
                                   speeds::armKinematicSpeeds::effectorAcceleration};
  auto safetyRaise = SetArmPoseCommand{m_lifter,
                                       m_bash,
                                       GetRelativePose(m_lifter.GetArmPose(), -6_in, 0_in),
                                       BashGuardPosition::Retracted,
                                       WristPosition::Unknown,
                                       PathType::unmodified,
                                       30_ips,
                                       speeds::armKinematicSpeeds::effectorAcceleration};

  if (m_lifter.GetArmPose().Y() > 20_in) {
    m_allCommands = frc2::ParallelCommandGroup(frc2::SequentialCommandGroup(gpScore, safetyRaise),
                                               frc2::SequentialCommandGroup(frc2::WaitCommand(750_ms), m_retractIntake))
                        .ToPtr();
  } else {
    m_allCommands =
        frc2::ParallelCommandGroup(frc2::InstantCommand{[this]() { m_intake.EjectConeForReal(); }, {&m_intake}},
                                   frc2::WaitCommand(750_ms))
            .ToPtr();
  }
  // Initialize all commands
  m_allCommands.Schedule();
}

// Called repeatedly when this Command is scheduled to run
void ScoreConeCommand::Execute() {
  if (m_lifter.IsFatalPathFault()) {
    Cancel();
    return;
  }
  if (!m_allCommands.IsScheduled()) {
    Cancel();
    return;
  }
}

// Called once the command ends or is interrupted.
void ScoreConeCommand::End(bool interrupted) {
  if (interrupted) {
    m_allCommands.Cancel();
  }
  m_intake.IntakeStop();
}

// Returns true when the command should end.
bool ScoreConeCommand::IsFinished() {
  return m_allCommands.get()->IsFinished();
}
