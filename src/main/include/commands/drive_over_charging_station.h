/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/swerve_drive_subsystem.h"

class DriveOverChargingStation : public frc2::CommandHelper<frc2::CommandBase, DriveOverChargingStation> {
 public:
  DriveOverChargingStation(SwerveDriveSubsystem* drive, units::degree_t approachAngle, units::degree_t robotYaw);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem* m_pDrive;         ///< Raw pointer to swerve drive subsystem object
  const units::degree_t m_approachAngle;  ///< Angle of drive direction relative to field-centric
  units::degree_t m_robotYawAngle;        ///< Robot orientation
  const bool m_approachForward;
  const int m_initialPitchSign;
  frc2::CommandPtr m_commands;  ///< Commands to execute
};
