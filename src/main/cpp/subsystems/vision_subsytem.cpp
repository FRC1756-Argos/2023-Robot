/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <frc/DriverStation.h>

#include "subsystems/vision_subsystem.h"

CameraInterface::CameraInterface() = default;

VisionSubsystem::VisionSubsystem(const argos_lib::RobotInstance instance, SwerveDriveSubsystem* pDriveSubsystem)
    : m_instance(instance), m_pDriveSubsystem(pDriveSubsystem) {}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
  LimelightTarget::tValues targetValues = GetCameraTargetValues();

  m_pDriveSubsystem->GetPoseEstimate(targetValues.robotPoseWPI.ToPose2d(), targetValues.totalLatency);
}

LimelightTarget::tValues VisionSubsystem::GetCameraTargetValues() {
  return m_cameraInterface.m_target.GetTarget();
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
  m_totalLatency = units::make_unit<units::millisecond_t>(rawRobotPose.at(6));

  return tValues{m_robotPose, m_robotPoseWPI, m_robotPoseTagSpace, m_hasTargets, m_totalLatency};
}

bool LimelightTarget::HasTarget() {
  return m_hasTargets;
}
