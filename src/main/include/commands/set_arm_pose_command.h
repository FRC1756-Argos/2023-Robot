/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Constants.h"
#include "constants/field_points.h"
#include "subsystems/bash_guard_subsystem.h"
#include "subsystems/lifter_subsystem.h"
#include "utils/custom_units.h"

enum class PathType { unmodified, concaveDown, concaveUp };

class SetArmPoseCommand : public frc2::CommandHelper<frc2::CommandBase, SetArmPoseCommand> {
 public:
  SetArmPoseCommand(LifterSubsystem* lifter,
                    BashGuardSubsystem* bashGuard,
                    std::function<ScoringPosition()> scoringPositionCb,
                    frc::Translation2d scoringPositionOffset,
                    std::function<bool()> bashGuardModeCb,
                    std::function<bool()> placeGamePieceInvertedCb,
                    PathType pathType = PathType::unmodified,
                    units::velocity::inches_per_second_t maxVelocity = speeds::armKinematicSpeeds::effectorVelocity,
                    units::acceleration::inches_per_second_squared_t maxAcceleration =
                        speeds::armKinematicSpeeds::effectorAcceleration);

  SetArmPoseCommand(LifterSubsystem* lifter,
                    BashGuardSubsystem* bashGuard,
                    ScoringPosition scoringPosition,
                    frc::Translation2d scoringPositionOffset,
                    std::function<bool()> bashGuardModeCb,
                    std::function<bool()> placeGamePieceInvertedCb,
                    PathType pathType = PathType::unmodified,
                    units::velocity::inches_per_second_t maxVelocity = speeds::armKinematicSpeeds::effectorVelocity,
                    units::acceleration::inches_per_second_squared_t maxAcceleration =
                        speeds::armKinematicSpeeds::effectorAcceleration);

  SetArmPoseCommand(LifterSubsystem* lifter,
                    BashGuardSubsystem* bashGuard,
                    frc::Translation2d targetPose,
                    frc::Translation2d scoringPositionOffset,
                    BashGuardPosition desiredBashGuardPosition,
                    WristPosition desiredWristPosition = WristPosition::Unknown,
                    PathType pathType = PathType::unmodified,
                    units::velocity::inches_per_second_t maxVelocity = speeds::armKinematicSpeeds::effectorVelocity,
                    units::acceleration::inches_per_second_squared_t maxAcceleration =
                        speeds::armKinematicSpeeds::effectorAcceleration,
                    bool isTunable = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  frc2::Command::InterruptionBehavior GetInterruptionBehavior() const override;

 private:
  LifterSubsystem* m_lifter;
  BashGuardSubsystem* m_bashGuard;
  std::optional<std::function<ScoringPosition()>> m_scoringPositionCb;
  frc::Translation2d m_scoringPositionOffset;
  std::optional<std::function<bool()>> m_bashGuardModeCb;
  std::optional<std::function<bool()>> m_placeGamePieceInvertedCb;
  frc::Translation2d m_targetPose;
  BashGuardPosition m_bashGuardTarget;
  units::velocity::inches_per_second_t m_maxVelocity;
  units::acceleration::inches_per_second_squared_t m_maxAcceleration;
  bool m_isTunable;

  ScoringPosition m_latestScoringPosition;
  bool m_lastInversion;
  PathType m_pathType;
  WristPosition m_endingWristPosition;

  bool m_hasShoulderMotion;
  bool m_hasExtensionMotion;
  bool m_hasBashGuardMotion;

  unsigned m_id;
};
