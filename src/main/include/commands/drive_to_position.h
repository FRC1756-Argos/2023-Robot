/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <memory>

#include "subsystems/swerve_drive_subsystem.h"
#include "commands/autonomous/autonomous_command.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveToPosition
    : public frc2::CommandHelper<frc2::CommandBase, DriveToPosition>
    , public AutonomousCommand {
 public:
  DriveToPosition(SwerveDriveSubsystem* drive,
                  const frc::Pose2d initPos,
                  const units::degree_t initAngle,
                  const frc::Pose2d dest,
                  const units::degree_t destAngle,
                  const frc::TrapezoidProfile<units::inches>::Constraints linConstraints,
                  frc::TrapezoidProfile<units::degrees>::Constraints rotConstraints);

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
  SwerveDriveSubsystem* m_drive;
  const frc::Pose2d m_initPosition;
  const units::degree_t m_initAngle;
  const frc::Pose2d m_destPosition;
  const units::degree_t m_destAngle;
  const frc::TrapezoidProfile<units::inches>::Constraints m_linearConstraints;
  const frc::TrapezoidProfile<units::degrees>::Constraints m_rotationalConstraints;
};
