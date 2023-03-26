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

SetArmPoseCommand::SetArmPoseCommand(LifterSubsystem* lifter,
                                     BashGuardSubsystem* bashGuard,
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
    , m_targetShoulder{0_deg}
    , m_targetExtension{0_in}
    , m_maxVelocity(maxVelocity)
    , m_maxAcceleration(maxAcceleration)
    , m_isTunable{false}
    , m_latestScoringPosition{}
    , m_lastInversion{false}
    , m_pathType{pathType}
    , m_endingWristPosition{WristPosition::Unknown}
    , m_hasShoulderMotion{false}
    , m_hasExtensionMotion{false}
    , m_hasBashGuardMotion{false}
    , m_isExtending{false}
    , m_wristSafeZoneMinAngle{m_lifter
                                  ->ConvertLifterPose(
                                      scoring_positions::lifter_extension_end::coneIntake.lifterPosition)
                                  .shoulderAngle +
                              5_deg}
    , m_id{static_cast<unsigned>(
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
              .count())} {
  AddRequirements(m_lifter);
  AddRequirements(m_bashGuard);
}

SetArmPoseCommand::SetArmPoseCommand(LifterSubsystem* lifter,
                                     BashGuardSubsystem* bashGuard,
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
    , m_targetShoulder{0_deg}
    , m_targetExtension{0_in}
    , m_maxVelocity(maxVelocity)
    , m_maxAcceleration(maxAcceleration)
    , m_isTunable{false}
    , m_latestScoringPosition{scoringPosition}
    , m_lastInversion{false}
    , m_pathType{pathType}
    , m_endingWristPosition{WristPosition::Unknown}
    , m_hasShoulderMotion{false}
    , m_hasExtensionMotion{false}
    , m_hasBashGuardMotion{false}
    , m_isExtending{false}
    , m_wristSafeZoneMinAngle{m_lifter
                                  ->ConvertLifterPose(
                                      scoring_positions::lifter_extension_end::coneIntake.lifterPosition)
                                  .shoulderAngle +
                              5_deg}
    , m_id{static_cast<unsigned>(
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
              .count())} {
  AddRequirements(m_lifter);
  AddRequirements(m_bashGuard);

  auto targetLifterPosition = GetTargetPosition(scoringPosition, true, m_endingWristPosition);

  if (targetLifterPosition) {
    m_targetPose = targetLifterPosition.value().lifterPosition;
    m_bashGuardTarget = targetLifterPosition.value().bashGuardPosition;
  } else {
    m_targetPose = m_lifter->GetArmPose();
  }
}

SetArmPoseCommand::SetArmPoseCommand(LifterSubsystem* lifter,
                                     BashGuardSubsystem* bashGuard,
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
    , m_targetShoulder{0_deg}
    , m_targetExtension{0_in}
    , m_maxVelocity(maxVelocity)
    , m_maxAcceleration(maxAcceleration)
    , m_isTunable{isTuneable}
    , m_latestScoringPosition{}
    , m_lastInversion{false}
    , m_pathType{pathType}
    , m_endingWristPosition{desiredWristPosition}
    , m_hasShoulderMotion{false}
    , m_hasExtensionMotion{false}
    , m_hasBashGuardMotion{false}
    , m_isExtending{false}
    , m_wristSafeZoneMinAngle{m_lifter
                                  ->ConvertLifterPose(
                                      scoring_positions::lifter_extension_end::coneIntake.lifterPosition)
                                  .shoulderAngle +
                              5_deg}
    , m_id{static_cast<unsigned>(
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
              .count())} {
  AddRequirements(m_lifter);
  AddRequirements(m_bashGuard);
}

// Called when the command is initially scheduled.
void SetArmPoseCommand::Initialize() {
  m_hasShoulderMotion = false;
  m_hasExtensionMotion = false;
  m_hasBashGuardMotion = false;

  auto bashGuardDisabled = m_bashGuard->GetHomeFailed();

  if ((!bashGuardDisabled && !m_bashGuard->IsBashGuardHomed()) || !m_lifter->IsArmExtensionHomed()) {
    Cancel();
    return;
  }

  m_lifter->ResetPathFaults();

  bool bashGuardEnable = !bashGuardDisabled && m_bashGuardModeCb ? m_bashGuardModeCb.value()() : true;

  // set latest scoring position if we are using callback
  if (m_scoringPositionCb) {
    m_latestScoringPosition = m_scoringPositionCb.value()();
  }

  switch (m_latestScoringPosition.column) {
    case ScoringColumn::coneIntake:
    case ScoringColumn::cubeIntake:
    case ScoringColumn::leftGrid_middleCube:
    case ScoringColumn::middleGrid_middleCube:
    case ScoringColumn::rightGrid_middleCube:
      m_endingWristPosition = WristPosition::RollersUp;
      break;
    case ScoringColumn::stow:
      if (m_scoringPositionCb) {
        m_endingWristPosition = WristPosition::RollersUp;
      } else {
        m_endingWristPosition = WristPosition::Unknown;
      }
      break;
    case ScoringColumn::invalid:
      m_endingWristPosition = WristPosition::Unknown;
      break;
    default:
      if constexpr (warning::nuclear::option::wristEnabled) {
        if (m_placeGamePieceInvertedCb) {
          m_endingWristPosition =
              m_placeGamePieceInvertedCb.value()() ? WristPosition::RollersDown : WristPosition::RollersUp;
        } else {
          m_endingWristPosition = WristPosition::Unknown;
        }
      } else {
        m_endingWristPosition = WristPosition::RollersUp;
      }
      break;
  }

  if (m_scoringPositionCb) {
    m_latestScoringPosition = m_scoringPositionCb.value()();
    auto targetLifterPosition = GetTargetPosition(m_latestScoringPosition, bashGuardEnable, m_endingWristPosition);

    if (targetLifterPosition) {
      m_targetPose = targetLifterPosition.value().lifterPosition;
      m_bashGuardTarget = targetLifterPosition.value().bashGuardPosition;
    } else {
      Cancel();
      return;
    }
  }

  units::inch_t targetBashGuardPosition =
      m_bashGuard->DecomposeBashExtension(bashGuardEnable ? m_bashGuardTarget : BashGuardPosition::Stationary);
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

  auto initialPosition = m_lifter->GetArmPose();

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
      default:
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
      default:
        // Don't insert extra points
        break;
    }
  }

  desiredPath.emplace_back(m_targetPose);

  frc::SmartDashboard::PutNumber("lifter/InitialX", units::inch_t(initialPosition.X()).to<double>());
  frc::SmartDashboard::PutNumber("lifter/InitialY", units::inch_t(initialPosition.Y()).to<double>());
  frc::SmartDashboard::PutNumber("lifter/DesiredX", units::inch_t(m_targetPose.X()).to<double>());
  frc::SmartDashboard::PutNumber("lifter/DesiredY", units::inch_t(m_targetPose.Y()).to<double>());

  auto bashGuardPath = path_planning::GenerateProfiledBashGuard(m_bashGuard->GetBashGuardExtension(),
                                                                targetBashGuardPosition,
                                                                {.maxVelocity = 120_ips, .maxAcceleration = 120_ips2},
                                                                50_ms);
  auto generalArmPath = path_planning::GenerateProfiledPath(
      desiredPath,
      {.maxVelocity = m_maxVelocity, .maxAcceleration = m_maxAcceleration},
      path_planning::Polygon(measure_up::PathPlanningKeepOutZone.begin(), measure_up::PathPlanningKeepOutZone.end()),
      50_ms);

  auto compositePath = path_planning::GenerateCompositeMPPath(
      generalArmPath, bashGuardPath, path_planning::ArmPathPoint(measure_up::lifter::fulcrumPosition), m_lifter);

  if (!bashGuardDisabled) {
    BufferedTrajectoryPointStream& bashGuardStream = m_bashGuard->GetMPStream();
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
  }

  BufferedTrajectoryPointStream& shoulderStream = m_lifter->GetShoulderMPStream();
  BufferedTrajectoryPointStream& armExtensionStream = m_lifter->GetExtensionMPStream();

  if (m_pathType != PathType::componentWise) {
    shoulderStream.Clear();
    for (auto pointIt = compositePath.shoulderPath.begin(); pointIt != compositePath.shoulderPath.end(); ++pointIt) {
      shoulderStream.Write(ctre::phoenix::motion::TrajectoryPoint(
          sensor_conversions::lifter::shoulder_actuator::ToSensorUnit(
              m_lifter->ConvertShoulderAngle(pointIt->position)),
          sensor_conversions::lifter::shoulder_actuator::ToSensorVelocity(
              -m_lifter->ConvertShoulderVelocity(pointIt->position, pointIt->velocity)),
          0,
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
  } else {
    // Initialization for component-wise path type

    shoulderStream.Clear();
    armExtensionStream.Clear();

    // Populate extension and shoulder targets
    auto jointPositions = m_lifter->ConvertLifterPose(m_targetPose);
    m_targetExtension = jointPositions.armLen;
    m_targetShoulder = jointPositions.shoulderAngle;

    // Extension extending or retracting dictates component-wise behavior
    if (m_targetExtension > m_lifter->GetArmExtension()) {
      m_isExtending = true;
    } else {
      m_isExtending = false;
    }

    frc::SmartDashboard::PutNumber("m_targetExtension:", m_targetExtension.to<double>());
    frc::SmartDashboard::PutNumber("m_targetShoulder:", m_targetShoulder.to<double>());

    m_lifter->SetShoulderManualOverride(false);
    m_lifter->SetExtensionManualOverride(false);

    // Stop any incorrect motion
    if (m_isExtending) {
      if (!m_lifter->IsShoulderAt(m_targetShoulder)) {
        m_lifter->StopArmExtension();
      }
    } else {
      if (!m_lifter->IsExtensionAt(m_targetExtension)) {
        m_lifter->StopShoulder();
      }
    }
  }

  m_hasBashGuardMotion = compositePath.bashGuardPath.size() > 0;
  m_hasShoulderMotion = compositePath.shoulderPath.size() > 0;
  m_hasExtensionMotion = compositePath.extensionPath.size() > 0;

  if (!bashGuardDisabled) {
    m_bashGuard->StartMotionProfile(compositePath.bashGuardPath.size());
  }
  if (m_pathType != PathType::componentWise) {
    m_lifter->StartMotionProfile(compositePath.shoulderPath.size(), compositePath.extensionPath.size(), 0);
  }
}

// Called repeatedly when this Command is scheduled to run
void SetArmPoseCommand::Execute() {
  if (m_bashGuard->IsBashGuardManualOverride() || m_lifter->IsExtensionManualOverride() ||
      m_lifter->IsShoulderManualOverride() || m_lifter->IsFatalPathFault()) {
    Cancel();
    return;
  }
  if (m_scoringPositionCb) {
    auto updatedScoringPosition = m_scoringPositionCb.value()();
    if (m_placeGamePieceInvertedCb) {
      auto updatedGamePieceInverted = m_placeGamePieceInvertedCb.value()();
      auto inversionChanged = updatedGamePieceInverted != m_lastInversion;
      m_lastInversion = updatedGamePieceInverted;
      if (inversionChanged && (m_latestScoringPosition.column != ScoringColumn::coneIntake &&
                               m_latestScoringPosition.column != ScoringColumn::cubeIntake &&
                               m_latestScoringPosition.column != ScoringColumn::stow &&
                               m_latestScoringPosition.column != ScoringColumn::leftGrid_middleCube &&
                               m_latestScoringPosition.column != ScoringColumn::middleGrid_middleCube &&
                               m_latestScoringPosition.column != ScoringColumn::rightGrid_middleCube)) {
        m_endingWristPosition = updatedGamePieceInverted ? WristPosition::RollersDown : WristPosition::RollersUp;
      }
    }
    if (updatedScoringPosition != m_latestScoringPosition) {
      m_lifter->StopMotionProfile();
      m_bashGuard->StopMotionProfile();
      // Check if bashguards enabled, if so, finish motion before re-initializing
      if (m_hasBashGuardMotion) {
        m_bashGuard->SetExtensionLength(m_bashGuard->DecomposeBashExtension(m_bashGuardTarget));
      }
      Initialize();
    }
  }

  if (m_pathType == PathType::componentWise) {
    if (m_isExtending) {
      // Shoulder first, then extension
      if (!m_lifter->IsShoulderAt(m_targetShoulder)) {
        m_lifter->SetShoulderAngle(m_targetShoulder);
      } else {
        m_lifter->SetArmExtension(m_targetExtension);
      }
    } else {
      // Extension first, then shoulder
      if (!m_lifter->IsExtensionAt(m_targetExtension)) {
        m_lifter->SetArmExtension(m_targetExtension);
      } else {
        m_lifter->SetShoulderAngle(m_targetShoulder);
      }
    }
  }

  // Move wrist when it's in a relatively safe area
  if (m_endingWristPosition != WristPosition::Unknown && m_lifter->GetArmExtension() < m_wristSafeZoneMaxExtension &&
      m_lifter->GetShoulderBoomAngle() >= m_wristSafeZoneMinAngle) {
    switch (m_endingWristPosition) {
      case WristPosition::RollersUp:
        m_lifter->SetWristAngle(measure_up::lifter::wrist::nominalAngle);
        break;
      case WristPosition::RollersDown:
        m_lifter->SetWristAngle(measure_up::lifter::wrist::invertedAngle);
        break;
      default:
        // Just leave wrist alone
        break;
    }
  }
}

// Called once the command ends or is interrupted.
void SetArmPoseCommand::End(bool interrupted) {
  if (interrupted) {
    m_lifter->StopMotionProfile();
    m_lifter->StopArmExtension();
    m_lifter->StopShoulder();
    m_bashGuard->StopMotionProfile();
    if (m_hasBashGuardMotion) {
      m_bashGuard->SetExtensionLength(m_bashGuard->DecomposeBashExtension(m_bashGuardTarget));
    }
  } else {
    switch (m_endingWristPosition) {
      case WristPosition::RollersUp:
        m_lifter->SetWristAngle(measure_up::lifter::wrist::nominalAngle);
        break;
      case WristPosition::RollersDown:
        m_lifter->SetWristAngle(measure_up::lifter::wrist::invertedAngle);
        break;
      default:
        // Just leave wrist alone
        break;
    }
    // m_lifter->SetLifterPose(m_targetPose, m_endingWristPosition);
  }
}

// Returns true when the command should end.
bool SetArmPoseCommand::IsFinished() {
  bool isFinished;

  if (m_pathType == PathType::componentWise) {
    isFinished = ((!m_hasBashGuardMotion || m_bashGuard->IsBashGuardMPComplete()) &&
                  m_lifter->IsShoulderAt(m_targetShoulder) && m_lifter->IsExtensionAt(m_targetExtension));
  } else {
    isFinished = (!m_hasExtensionMotion || m_lifter->IsExtensionMPComplete()) &&
                 (!m_hasShoulderMotion || m_lifter->IsShoulderMPComplete()) &&
                 (!m_hasBashGuardMotion || m_bashGuard->IsBashGuardMPComplete());
  }

  frc::SmartDashboard::PutBoolean("Set Arm Pose Command Is Finished: ", isFinished);
  return isFinished;
}

frc2::Command::InterruptionBehavior SetArmPoseCommand::GetInterruptionBehavior() const {
  return InterruptionBehavior::kCancelSelf;
}
