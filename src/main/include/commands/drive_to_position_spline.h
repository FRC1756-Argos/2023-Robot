/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/spline/Spline.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <memory>
#include <vector>

#include "subsystems/swerve_drive_subsystem.h"
#include "utils/swerve_trapezoidal_spline.h"

class DriveToPositionSpline : public frc2::CommandHelper<frc2::CommandBase, DriveToPositionSpline> {
 public:
  DriveToPositionSpline(SwerveDriveSubsystem* drive,
                        const frc::Spline<3>::ControlVector initialPosition,
                        const units::degree_t initialAngle,
                        const std::vector<frc::Translation2d>& waypoints,
                        const frc::Spline<3>::ControlVector finalPosition,
                        const units::degree_t finalAngle,
                        const units::second_t turnDelay,
                        const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints,
                        const frc::TrapezoidProfile<units::degrees>::Constraints rotationalConstraints,
                        const units::feet_per_second_t initialVelocity = 0_fps,
                        const units::feet_per_second_t finalVelocity = 0_fps);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem* m_pDrive;
  SwerveTrapezoidalSpline m_splinePath;
  const units::degree_t m_initialAngle;
  const units::degree_t m_finalAngle;
  const frc::TrapezoidProfile<units::inches>::Constraints m_linearConstraints;
  const frc::TrapezoidProfile<units::degrees>::Constraints m_rotationalConstraints;
  const units::feet_per_second_t m_initialVelocity;
  const units::feet_per_second_t m_finalVelocity;
};

frc::Spline<3>::ControlVector ConvertToControlVector(units::foot_t x,
                                                     units::foot_t y,
                                                     units::foot_t tangentX,
                                                     units::foot_t tangentY);
