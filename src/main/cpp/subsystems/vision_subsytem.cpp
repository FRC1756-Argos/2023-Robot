/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "subsystems/vision_subsystem.h"

CameraInterface::CameraInterface() = default;

VisionSubsystem::VisionSubsystem(const argos_lib::RobotInstance instance, SwerveDriveSubsystem* pDriveSubsystem)
    : m_instance(instance), m_pDriveSubsystem(pDriveSubsystem) {}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
  LimelightTarget::tValues targetValues = GetCameraTargetValues();  // Note that this will update the targets object

  if (targetValues.hasTargets &&
      (targetValues.robotPose.ToPose2d().X() != 0_in && targetValues.robotPose.ToPose2d().Y() != 0_in)) {
    // m_pDriveSubsystem->GetPoseEstimate(targetValues.robotPoseWPI.ToPose2d(), targetValues.totalLatency);
  }

  GetDistanceToPoleTape();

  if (targetValues.hasTargets) {
    frc::SmartDashboard::PutBoolean("(Vision - Periodic) Is Target Present?", targetValues.hasTargets);
    frc::SmartDashboard::PutNumber("(Vision - Periodic) Target Pitch", targetValues.m_pitch.to<double>());
    frc::SmartDashboard::PutNumber("(Vision - Periodic) Target Yaw", targetValues.m_yaw.to<double>());
  }
}

std::optional<units::degree_t> VisionSubsystem::GetHorizontalOffsetToTarget() {
  // Updates and retrieves new target values
  LimelightTarget::tValues targetValues = GetCameraTargetValues();

  // vision debugs
  frc::SmartDashboard::PutBoolean("(AimToPlaceCone) Is Target Present?", targetValues.hasTargets);
  frc::SmartDashboard::PutNumber("(AimToPlaceCone) Target Pitch", targetValues.m_pitch.to<double>());
  frc::SmartDashboard::PutNumber("(AimToPlaceCone) Target Yaw", targetValues.m_yaw.to<double>());

  // add more target validation after testing e.g. area, margin, skew etc
  // for now has target is enough as we will be fairly close to target
  // and will tune the pipeline not to combine detections and choose the highest area
  if (targetValues.hasTargets) {
    return targetValues.m_yaw;
  }

  return std::nullopt;
}

std::optional<units::inch_t> VisionSubsystem::GetDistanceToPoleTape() {
  const auto targets = GetCameraTargetValues();
  if (targets.hasTargets) {
    units::inch_t distance;

    // first of all, change the pipeline to sort the targets "closest" and distance we return should always be to the bottom pole
    // if we determine target we are seeing is the upper pole based on the properties, then calculate accordingly
    // else assume we are seeing lower target all the time

    const units::radian_t targetAngle =
        measure_up::camera::cameraMountAngle - (measure_up::camera::vFov / 2) + targets.m_pitch;

    // Assume we're seeing the low node, because robot can quickly drive away from grid if it
    // incorrectly assumes high node
    distance =
        (measure_up::camera::bottomPoleTapeCenter - measure_up::camera::cameraZ) / std::tan(targetAngle.to<double>()) +
        measure_up::camera::cameraX;

    frc::SmartDashboard::PutNumber("(GetDistanceToPoleTape) Vision Distance To LowerPole RetroReflective Tape (inches)",
                                   distance.to<double>());

    return distance;
  }

  return std::nullopt;
}

void VisionSubsystem::SetReflectiveVisionMode(bool mode) {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  int requestedPipeline = mode ? camera::reflectivePipeline : camera::aprilTagPipeline;

  frc::SmartDashboard::PutNumber("(SetReflectiveVisionMode) Pipeline", requestedPipeline);

  table->PutNumber("pipeline", requestedPipeline);
}

/// @todo deprecate?
bool VisionSubsystem::AimToPlaceCone() {
  return true;
}

LimelightTarget::tValues VisionSubsystem::GetCameraTargetValues() {
  return m_cameraInterface.m_target.GetTarget();
}

void VisionSubsystem::Disable() {
  SetReflectiveVisionMode(false);
}

// LIMELIGHT TARGET MEMBER FUNCTIONS ===============================================================

LimelightTarget::tValues LimelightTarget::GetTarget() {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  auto rawRobotPose = table->GetNumberArray("botpose", std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_robotPose = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(rawRobotPose.at(0)),
                                               units::make_unit<units::meter_t>(rawRobotPose.at(1)),
                                               units::make_unit<units::meter_t>(rawRobotPose.at(2))),
                            frc::Rotation3d(units::make_unit<units::radian_t>(rawRobotPose.at(3)),
                                            units::make_unit<units::radian_t>(rawRobotPose.at(4)),
                                            units::make_unit<units::radian_t>(rawRobotPose.at(5))));
  auto rawRobotPoseWPI = table->GetNumberArray(
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ? "botpose_wpiblue" : "botpose_wpired",
      std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_robotPoseWPI = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(rawRobotPoseWPI.at(0)),
                                                  units::make_unit<units::meter_t>(rawRobotPoseWPI.at(1)),
                                                  units::make_unit<units::meter_t>(rawRobotPoseWPI.at(2))),
                               frc::Rotation3d(units::make_unit<units::radian_t>(rawRobotPoseWPI.at(3)),
                                               units::make_unit<units::radian_t>(rawRobotPoseWPI.at(4)),
                                               units::make_unit<units::radian_t>(rawRobotPoseWPI.at(5))));
  auto rawRobotTagSpace =
      table->GetNumberArray("botpose_targetspace", std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_robotPoseTagSpace = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(rawRobotTagSpace.at(0)),
                                                       units::make_unit<units::meter_t>(rawRobotTagSpace.at(1)),
                                                       units::make_unit<units::meter_t>(rawRobotTagSpace.at(2))),
                                    frc::Rotation3d(units::make_unit<units::radian_t>(rawRobotTagSpace.at(3)),
                                                    units::make_unit<units::radian_t>(rawRobotTagSpace.at(4)),
                                                    units::make_unit<units::radian_t>(rawRobotTagSpace.at(5))));
  m_hasTargets = (table->GetNumber("tv", 0) == 1);
  m_yaw = units::make_unit<units::degree_t>(table->GetNumber("tx", 0.0));
  m_pitch = units::make_unit<units::degree_t>(table->GetNumber("ty", 0.0));
  m_area = (table->GetNumber("ta", 0.0));
  m_totalLatency = units::make_unit<units::millisecond_t>(rawRobotPose.at(6));

  return tValues{
      m_robotPose, m_robotPoseWPI, m_robotPoseTagSpace, m_hasTargets, m_pitch, m_yaw, m_area, m_totalLatency};
}

bool LimelightTarget::HasTarget() {
  return m_hasTargets;
}
