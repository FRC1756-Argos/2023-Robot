/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/set_arm_pose_command.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "constants/measure_up.h"
#include "constants/scoring_positions.h"
#include "ctre/phoenix/motion/BufferedTrajectoryPointStream.h"
#include "utils/path_planning/convert_path.h"
#include "utils/sensor_conversions.h"

SetArmPoseCommand::SetArmPoseCommand(LifterSubsystem& lifter,
                                     BashGuardSubsystem& bashGuard,
                                     std::function<ScoringPosition()> scoringPositionCb,
                                     std::function<bool()> bashGuardModeCb,
                                     std::function<bool()> placeGamePieceInvertedCb,
                                     PathType pathType,
                                     units::inches_per_second_t maxVelocity,
                                     units::inches_per_second_squared_t maxAcceleration)
    : m_lifter(lifter)
    , m_bashGuard(bashGuard)
    , m_scoringPositionCb(scoringPositionCb)
    , m_bashGuardModeCb(bashGuardModeCb)
    , m_placeGamePieceInvertedCb(placeGamePieceInvertedCb)
    , m_targetPose()
    , m_bashGuardTarget()
    , m_maxVelocity(maxVelocity)
    , m_maxAcceleration(maxAcceleration)
    , m_isTunable{false}
    , m_latestScoringPosition{}
    , m_pathType{pathType}
    , m_endingWristPosition{WristPosition::Unknown}
    , m_hasShoulderMotion{false}
    , m_hasExtensionMotion{false}
    , m_hasBashGuardMotion{false}
    , m_id{static_cast<unsigned>(
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
              .count())} {
  AddRequirements(&m_lifter);
  AddRequirements(&m_bashGuard);
}

SetArmPoseCommand::SetArmPoseCommand(LifterSubsystem& lifter,
                                     BashGuardSubsystem& bashGuard,
                                     ScoringPosition scoringPosition,
                                     std::function<bool()> bashGuardModeCb,
                                     std::function<bool()> placeGamePieceInvertedCb,
                                     PathType pathType,
                                     units::inches_per_second_t maxVelocity,
                                     units::inches_per_second_squared_t maxAcceleration)
    : m_lifter(lifter)
    , m_bashGuard(bashGuard)
    , m_scoringPositionCb(std::nullopt)
    , m_bashGuardModeCb(bashGuardModeCb)
    , m_placeGamePieceInvertedCb(placeGamePieceInvertedCb)
    , m_targetPose()
    , m_bashGuardTarget()
    , m_maxVelocity(maxVelocity)
    , m_maxAcceleration(maxAcceleration)
    , m_isTunable{false}
    , m_latestScoringPosition{scoringPosition}
    , m_pathType{pathType}
    , m_endingWristPosition{WristPosition::Unknown}
    , m_hasShoulderMotion{false}
    , m_hasExtensionMotion{false}
    , m_hasBashGuardMotion{false}
    , m_id{static_cast<unsigned>(
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
              .count())} {
  AddRequirements(&m_lifter);
  AddRequirements(&m_bashGuard);

  auto targetEffectorPosition = GetTargetPosition(scoringPosition, true);

  if (targetEffectorPosition) {
    m_targetPose = targetEffectorPosition.value().endEffectorPosition;
    m_bashGuardTarget = targetEffectorPosition.value().bashGuardPosition;
  } else {
    m_targetPose = m_lifter.GetArmPose(WristPosition::RollersUp);
  }
}

SetArmPoseCommand::SetArmPoseCommand(LifterSubsystem& lifter,
                                     BashGuardSubsystem& bashGuard,
                                     frc::Translation2d targetPose,
                                     BashGuardPosition desiredBashGuardPosition,
                                     WristPosition desiredWristPosition,
                                     PathType pathType,
                                     units::inches_per_second_t maxVelocity,
                                     units::inches_per_second_squared_t maxAcceleration,
                                     bool isTuneable)
    : m_lifter(lifter)
    , m_bashGuard(bashGuard)
    , m_scoringPositionCb(std::nullopt)
    , m_bashGuardModeCb(std::nullopt)
    , m_placeGamePieceInvertedCb(std::nullopt)
    , m_targetPose(targetPose)
    , m_bashGuardTarget(desiredBashGuardPosition)
    , m_maxVelocity(maxVelocity)
    , m_maxAcceleration(maxAcceleration)
    , m_isTunable{isTuneable}
    , m_latestScoringPosition{}
    , m_pathType{pathType}
    , m_endingWristPosition{desiredWristPosition}
    , m_hasShoulderMotion{false}
    , m_hasExtensionMotion{false}
    , m_hasBashGuardMotion{false}
    , m_id{static_cast<unsigned>(
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
              .count())} {
  AddRequirements(&m_lifter);
  AddRequirements(&m_bashGuard);
}

// Called when the command is initially scheduled.
void SetArmPoseCommand::Initialize() {
  m_hasShoulderMotion = false;
  m_hasExtensionMotion = false;
  m_hasBashGuardMotion = false;

  if (!m_bashGuard.IsBashGuardHomed() || !m_lifter.IsArmExtensionHomed()) {
    Cancel();
    return;
  }

  bool bashGuardEnable = m_bashGuardModeCb ? m_bashGuardModeCb.value()() : true;

  if (m_scoringPositionCb) {
    m_latestScoringPosition = m_scoringPositionCb.value()();
    auto targetEffectorPosition = GetTargetPosition(m_latestScoringPosition, bashGuardEnable);

    if (targetEffectorPosition) {
      m_targetPose = targetEffectorPosition.value().endEffectorPosition;
      m_bashGuardTarget = targetEffectorPosition.value().bashGuardPosition;
    } else {
      Cancel();
      return;
    }
  }

  switch (m_latestScoringPosition.column) {
    case ScoringColumn::intake:
      m_endingWristPosition = WristPosition::RollersUp;
      break;
    case ScoringColumn::stow:
    case ScoringColumn::invalid:
      m_endingWristPosition = WristPosition::Unknown;
      break;
    default:
      if (m_placeGamePieceInvertedCb) {
        m_endingWristPosition =
            m_placeGamePieceInvertedCb.value()() ? WristPosition::RollersDown : WristPosition::RollersUp;
      } else {
        m_endingWristPosition = WristPosition::Unknown;
      }
      break;
  }

  units::inch_t targetBashGuardPosition =
      m_bashGuard.DecomposeBashExtension(bashGuardEnable ? m_bashGuardTarget : BashGuardPosition::Stationary);
  // For testing, load all these during initialization so we can adjust
  if (m_isTunable) {
    targetBashGuardPosition =
        units::make_unit<units::inch_t>((frc::SmartDashboard::GetNumber("MPTesting/BashGuard", 0)));
    m_targetPose = frc::Translation2d(
        units::make_unit<units::inch_t>(frc::SmartDashboard::GetNumber("MPTesting/TargetX (in)", 50.0)),
        units::make_unit<units::inch_t>(frc::SmartDashboard::GetNumber("MPTesting/TargetZ (in)", 18.0)));

    m_maxVelocity = units::make_unit<units::inches_per_second_t>(
        frc::SmartDashboard::GetNumber("MPTesting/TravelSpeed (in/s)", 90.0));
    m_maxAcceleration = units::make_unit<units::inches_per_second_squared_t>(
        frc::SmartDashboard::GetNumber("MPTesting/TravelAccel (in/s^2)", 80.0));
  }

  switch (m_endingWristPosition) {
    case WristPosition::RollersDown:
      frc::SmartDashboard::PutString("lifter/DesiredWrist", "Rollers Down");
      break;
    case WristPosition::RollersUp:
      frc::SmartDashboard::PutString("lifter/DesiredWrist", "Rollers Up");
      break;
    case WristPosition::Unknown:
      frc::SmartDashboard::PutString("lifter/DesiredWrist", "Unknown");
      break;
  }

  auto initialPosition = m_lifter.GetArmPose(m_endingWristPosition);

  path_planning::ArmPath desiredPath;
  desiredPath.reserve(3);
  desiredPath.emplace_back(initialPosition);

  auto maxY = units::math::max(m_targetPose.Y(), initialPosition.Y());
  auto minY = units::math::min(m_targetPose.Y(), initialPosition.Y());

  bool upwardMotion = m_targetPose.Y() > initialPosition.Y();

  if (upwardMotion) {
    switch (m_pathType) {
      case PathType::concaveDown:
        desiredPath.emplace_back(initialPosition.X(), maxY);
        break;
      case PathType::concaveUp:
        desiredPath.emplace_back(m_targetPose.X(), minY);
        break;
      case PathType::unmodified:
        // Don't insert extra points
        break;
    }
  } else {
    switch (m_pathType) {
      case PathType::concaveDown:
        desiredPath.emplace_back(m_targetPose.X(), maxY);
        break;
      case PathType::concaveUp:
        desiredPath.emplace_back(initialPosition.X(), minY);
        break;
      case PathType::unmodified:
        // Don't insert extra points
        break;
    }
  }

  desiredPath.emplace_back(m_targetPose);

  frc::SmartDashboard::PutNumber("lifter/InitialX", units::inch_t(initialPosition.X()).to<double>());
  frc::SmartDashboard::PutNumber("lifter/InitialY", units::inch_t(initialPosition.Y()).to<double>());
  frc::SmartDashboard::PutNumber("lifter/DesiredX", units::inch_t(m_targetPose.X()).to<double>());
  frc::SmartDashboard::PutNumber("lifter/DesiredY", units::inch_t(m_targetPose.Y()).to<double>());

  auto bashGuardPath =
      path_planning::GenerateProfiledBashGuard(m_bashGuard.GetBashGuardExtension(),
                                               targetBashGuardPosition,
                                               {.maxVelocity = m_maxVelocity, .maxAcceleration = m_maxAcceleration},
                                               50_ms);
  auto generalArmPath = path_planning::GenerateProfiledPath(
      desiredPath,
      {.maxVelocity = m_maxVelocity, .maxAcceleration = m_maxAcceleration},
      path_planning::Polygon(measure_up::PathPlanningKeepOutZone.begin(), measure_up::PathPlanningKeepOutZone.end()),
      50_ms);

  auto compositePath =
      path_planning::GenerateCompositeMPPath(generalArmPath,
                                             bashGuardPath,
                                             path_planning::ArmPathPoint(measure_up::lifter::fulcrumPosition),
                                             m_lifter,
                                             m_endingWristPosition);

  BufferedTrajectoryPointStream& bashGuardStream = m_bashGuard.GetMPStream();
  bashGuardStream.Clear();
  for (auto pointIt = compositePath.bashGuardPath.begin(); pointIt != compositePath.bashGuardPath.end(); ++pointIt) {
    bashGuardStream.Write(
        ctre::phoenix::motion::TrajectoryPoint(sensor_conversions::bashguard::ToSensorUnit(pointIt->position),
                                               sensor_conversions::bashguard::ToSensorVelocity(pointIt->velocity),
                                               0,
                                               0,
                                               0,
                                               0,
                                               0,
                                               0,
                                               pointIt == std::prev(compositePath.bashGuardPath.end()),
                                               false,
                                               pointIt->time.to<double>(),
                                               false));
  }

  BufferedTrajectoryPointStream& shoulderStream = m_lifter.GetShoulderMPStream();
  shoulderStream.Clear();
  for (auto pointIt = compositePath.shoulderPath.begin(); pointIt != compositePath.shoulderPath.end(); ++pointIt) {
    shoulderStream.Write(ctre::phoenix::motion::TrajectoryPoint(
        sensor_conversions::lifter::shoulder::ToSensorUnit(pointIt->position),
        sensor_conversions::lifter::shoulder::ToSensorVelocity(-pointIt->velocity),
        0,  // pointIt->velocity < 0_rpm ? 0.001 * sensor_conversions::lifter::shoulder::ToSensorVelocity(-pointIt->velocity) :
        //                             0.0,
        0,
        0,
        0,
        0,
        0,
        pointIt == std::prev(compositePath.shoulderPath.end()),
        false,
        pointIt->time.to<double>(),
        false));
  }

  BufferedTrajectoryPointStream& armExtensionStream = m_lifter.GetExtensionMPStream();
  armExtensionStream.Clear();
  for (auto pointIt = compositePath.extensionPath.begin(); pointIt != compositePath.extensionPath.end(); ++pointIt) {
    armExtensionStream.Write(ctre::phoenix::motion::TrajectoryPoint(
        sensor_conversions::lifter::arm_extension::ToSensorUnit(pointIt->position),
        sensor_conversions::lifter::arm_extension::ToSensorVelocity(pointIt->velocity),
        0,
        0,
        0,
        0,
        0,
        0,
        pointIt == std::prev(compositePath.extensionPath.end()),
        false,
        pointIt->time.to<double>(),
        false));
  }

  m_hasBashGuardMotion = compositePath.bashGuardPath.size() > 0;
  m_hasShoulderMotion = compositePath.shoulderPath.size() > 0;
  m_hasExtensionMotion = compositePath.extensionPath.size() > 0;

  m_bashGuard.StartMotionProfile(compositePath.bashGuardPath.size());
  m_lifter.StartMotionProfile(compositePath.shoulderPath.size(), compositePath.extensionPath.size(), 0);
}

// Called repeatedly when this Command is scheduled to run
void SetArmPoseCommand::Execute() {
  if (m_bashGuard.IsBashGuardManualOverride() || m_lifter.IsExtensionManualOverride() ||
      m_lifter.IsShoulderManualOverride()) {
    Cancel();
    return;
  }
  if (m_scoringPositionCb) {
    auto updatedScoringPosition = m_scoringPositionCb.value()();
    if (updatedScoringPosition != m_latestScoringPosition) {
      m_lifter.StopMotionProfile();
      m_bashGuard.StopMotionProfile();
      Initialize();
    }
  }
}

// Called once the command ends or is interrupted.
void SetArmPoseCommand::End(bool interrupted) {
  if (interrupted) {
    m_lifter.StopMotionProfile();
    m_bashGuard.StopMotionProfile();
  } else {
    switch (m_endingWristPosition) {
      case WristPosition::RollersUp:
        m_lifter.SetWristAngle(measure_up::lifter::wrist::nominalAngle);
        break;
      case WristPosition::RollersDown:
        m_lifter.SetWristAngle(measure_up::lifter::wrist::invertedAngle);
        break;
      default:
        // Just leave wrist alone
        break;
    }
    // m_lifter.SetLifterPose(m_targetPose, m_endingWristPosition);
  }
}

// Returns true when the command should end.
bool SetArmPoseCommand::IsFinished() {
  return (!m_hasExtensionMotion || m_lifter.IsExtensionMPComplete()) &&
         (!m_hasShoulderMotion || m_lifter.IsShoulderMPComplete()) &&
         (!m_hasBashGuardMotion || m_bashGuard.IsBashGuardMPComplete());
}

frc2::Command::InterruptionBehavior SetArmPoseCommand::GetInterruptionBehavior() const {
  return InterruptionBehavior::kCancelSelf;
}
