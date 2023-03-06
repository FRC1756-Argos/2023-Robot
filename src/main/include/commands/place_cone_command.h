/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <commands/autonomous/autonomous_command.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <subsystems/bash_guard_subsystem.h>
#include <subsystems/lifter_subsystem.h>
#include <subsystems/intake_subsystem.h>
#include <subsystems/simple_led_subsystem.h>
#include <subsystems/swerve_drive_subsystem.h>

#include <string>

class PlaceConeCommand
    : public frc2::CommandHelper<frc2::CommandBase, PlaceConeCommand>
    , public AutonomousCommand {
 public:
  PlaceConeCommand(BashGuardSubsystem& bash,
                   LifterSubsystem& lifter,
                   SimpleLedSubsystem& leds,
                   IntakeSubsystem& intake,
                   frc::Translation2d desiredArmPos,
                   ScoringPosition scoringPos);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  /**
   * @copydoc AutonomousCommand::GetName()
   */
  std::string GetName() const final;
  /**
   * @copydoc AutonomousCommand::GetCommand()
   */
  frc2::Command* GetCommand() final;

 private:
  static units::degree_t GetShoulderAngle(const LifterSubsystem& lifter, const frc::Translation2d& pose) {
    units::degree_t shoulderAngle = lifter.ConvertLifterPose(pose).shoulderAngle;
    frc::SmartDashboard::PutNumber("(GetShoulderAngle) degrees", units::degree_t(shoulderAngle).to<double>());
    return shoulderAngle;
  }

  BashGuardSubsystem& m_bashGuard;
  LifterSubsystem& m_lifter;
  SimpleLedSubsystem& m_leds;
  IntakeSubsystem& m_intake;

  frc::Translation2d m_desiredArmPos;
  ScoringPosition m_scoringPosition;

  frc2::CommandPtr m_allCommands;
};
