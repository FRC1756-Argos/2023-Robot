/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "constants/field_points.h"
#include "subsystems/bash_guard_subsystem.h"
#include "subsystems/lifter_subsystem.h"
#include "utils/custom_units.h"

class SetArmPoseCommand : public frc2::CommandHelper<frc2::CommandBase, SetArmPoseCommand> {
 public:
  SetArmPoseCommand(LifterSubsystem& lifter,
                    BashGuardSubsystem& bashGuard,
                    std::function<ScoringPosition()> scoringPositionCb,
                    std::function<bool()> bashGuardModeCb,
                    units::velocity::inches_per_second_t maxVelocity = 90_ips,
                    units::acceleration::inches_per_second_squared_t maxAcceleration = 90_ips2);

  SetArmPoseCommand(LifterSubsystem& lifter,
                    BashGuardSubsystem& bashGuard,
                    frc::Translation2d targetPose,
                    BashGuardPosition desiredBashGuardPosition,
                    units::velocity::inches_per_second_t maxVelocity,
                    units::acceleration::inches_per_second_squared_t maxAcceleration,
                    bool isTunable);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  LifterSubsystem& m_lifter;
  BashGuardSubsystem& m_bashGuard;
  std::optional<std::function<ScoringPosition()>> m_scoringPositionCb;
  std::optional<std::function<bool()>> m_bashGuardModeCb;
  frc::Translation2d m_targetPose;
  BashGuardPosition m_bashGuardTarget;
  units::velocity::inches_per_second_t m_maxVelocity;
  units::acceleration::inches_per_second_squared_t m_maxAcceleration;
  bool m_isTunable;
};
