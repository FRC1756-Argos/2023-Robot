/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/set_arm_pose_command.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "constants/measure_up.h"
#include "ctre/phoenix/motion/BufferedTrajectoryPointStream.h"
#include "utils/path_planning/convert_path.h"
#include "utils/sensor_conversions.h"

SetArmPoseCommand::SetArmPoseCommand(LifterSubsystem& lifter,
                                     BashGuardSubsystem& bashGuard,
                                     frc::Translation2d targetPose,
                                     BashGuardPosition desiredBashGuardPosition,
                                     units::inches_per_second_t maxVelocity,
                                     units::inches_per_second_squared_t maxAcceleration)
    : m_lifter(lifter)
    , m_bashGuard(bashGuard)
    , m_targetPose(targetPose)
    , m_bashGuardTarget(desiredBashGuardPosition)
    , m_maxVelocity(maxVelocity)
    , m_maxAcceleration(maxAcceleration) {
  AddRequirements(&m_lifter);
  AddRequirements(&m_bashGuard);
}

// Called when the command is initially scheduled.
void SetArmPoseCommand::Initialize() {
  if (!m_bashGuard.IsBashGuardHomed() || !m_lifter.IsArmExtensionHomed()) {
    Cancel();
  }

  // For testing, load all these during initialization so we can adjust
  m_bashGuardTarget = static_cast<BashGuardPosition>(frc::SmartDashboard::GetNumber("MPTesting/BashGuard", 0));
  m_targetPose = frc::Translation2d(
      units::make_unit<units::inch_t>(frc::SmartDashboard::GetNumber("MPTesting/TargetX (in)", 50.0)),
      units::make_unit<units::inch_t>(frc::SmartDashboard::GetNumber("MPTesting/TargetY (in)", 18.0)));

  m_maxVelocity = units::make_unit<units::inches_per_second_t>(
      frc::SmartDashboard::GetNumber("MPTesting/TravelSpeed (in/s)", 120.0));
  m_maxAcceleration = units::make_unit<units::inches_per_second_squared_t>(
      frc::SmartDashboard::GetNumber("MPTesting/TravelAccel (in/s^2)", 120.0));

  auto initialPosition = m_lifter.GetArmPose();

  units::inch_t targetBashGuardPosition;
  switch (m_bashGuardTarget) {
    case BashGuardPosition::Deployed:
      targetBashGuardPosition = measure_up::bash::deployedExtension;
      break;
    case BashGuardPosition::Retracted:
      targetBashGuardPosition = measure_up::bash::retractedExtension;
      break;
    case BashGuardPosition::Stationary:
    default:
      targetBashGuardPosition = m_bashGuard.GetBashGuardExtension();
      break;
  }
  auto bashGuardPath =
      path_planning::GenerateProfiledBashGuard(m_bashGuard.GetBashGuardExtension(),
                                               targetBashGuardPosition,
                                               {.maxVelocity = m_maxVelocity, .maxAcceleration = m_maxAcceleration});
  auto generalArmPath = path_planning::GenerateProfiledPath(
      path_planning::ArmPathPoint(initialPosition),
      path_planning::ArmPathPoint(m_targetPose),
      {.maxVelocity = m_maxVelocity, .maxAcceleration = m_maxAcceleration},
      path_planning::Polygon(measure_up::PathPlanningKeepOutZone.begin(), measure_up::PathPlanningKeepOutZone.end()));

  auto compositePath = path_planning::GenerateCompositeMPPath(
      generalArmPath, bashGuardPath, path_planning::ArmPathPoint(measure_up::lifter::fulcrumPosition), m_lifter);

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
        sensor_conversions::lifter::shoulder::ToSensorVelocity(pointIt->velocity),
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

  m_bashGuard.StartMotionProfile();
  m_lifter.StartMotionProfile();
}

// Called repeatedly when this Command is scheduled to run
void SetArmPoseCommand::Execute() {
  if (m_bashGuard.IsBashGuardManualOverride() || m_lifter.IsExtensionManualOverride() ||
      m_lifter.IsShoulderManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void SetArmPoseCommand::End(bool interrupted) {
  if (interrupted) {
    m_lifter.StopArmExtension();
    m_lifter.StopShoulder();
    m_bashGuard.Stop();
  }
}

// Returns true when the command should end.
bool SetArmPoseCommand::IsFinished() {
  return m_lifter.IsExtensionMPComplete() && m_lifter.IsShoulderMPComplete() && m_bashGuard.IsBashGuardMPComplete();
}
