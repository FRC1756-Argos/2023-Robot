/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc2/command/SubsystemBase.h>

#include "argos_lib/config/config_types.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

class LimelightTarget {
 private:
  frc::Pose3d m_targetPose;                ///< 3d pose of target in view
  frc::Transform3d m_targetToRobot;        ///< 3d transform of target in view in robot space
  int m_targetId;                          ///< April tag id of the target in view
  units::degree_t m_pitch;                 ///< Pitch of target relative to camera -20.5 to 20.5 degrees
  units::degree_t m_yaw;                   ///< Yaw of target relative to camera -27 to 27 degrees
  bool m_hasTargets;                       ///< True if the camera has a target it can read
  units::millisecond_t m_pipelineLatency;  ///< Pipeline latency contribution
  constexpr static units::millisecond_t m_miscLatency{11_ms};  ///< Any extra latency to account for

 public:
  LimelightTarget() = default;

  /**
   * @brief Wraps members of LimelightTarget for use elsewhere
   *
   */
  struct tValues {
    units::degree_t pitch;                    ///< See LimelightTarget::m_pitch
    units::degree_t yaw;                      ///< See LimelightTarget::m_yaw
    frc::Pose3d targetPose;                   ///< See LimelightTarget::m_targetPose
    frc::Transform3d targetToRobotTransform;  ///< See LimelightTarget::m_targetToRobot
    int targetId;                             ///< See LimelightTarget::m_targetId
    units::millisecond_t totalLatency;        ///< See LimelightTarget::m_pipelineLatency
  };

  /**
   * @brief Get the values of the camera's current target
   *
   * @return tValues
   */
  tValues GetTarget();

  /**
   * @brief Does the camera see a target?
   *
   * @return true - The camera does see a target
   * @return false - The camera does not see a target
   */
  bool HasTarget();
};

/**
 * @brief Provides methods for interacting with the camera on a high level
 *
 */
class CameraInterface {
 public:
  CameraInterface();

  LimelightTarget m_target;  ///< object that holds the current target seen by the camera

  /**
   * @brief Get the closest target the camera can see CAN RETURN NONE
   *
   * @return std::optional<photonlib::PhotonTrackedTarget>
   */
  std::optional<LimelightTarget> GetClosestTarget();

  /**
   * @brief Turns the camera's driver mode on and off
   *
   * @param mode True is drive control. False is no drive control
   */
  void SetDriverMode(bool mode);
};

class vision_subsytem : public frc2::SubsystemBase {
 public:
  vision_subsytem();

  /**
   * @brief Get the offset to the center of the target
   *
   * @param target Target to aim toward
   * @return Desired offset.
   */
  std::optional<units::degree_t> GetOffsetToTarget(LimelightTarget::tValues target);

  /**
   * @brief Get the pitch and yaw of target
   *
   * @return LimelightTarget::tValues
   */
  LimelightTarget::tValues GetCameraTargetValues();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  CameraInterface m_cameraInterface;  ///< Interface to limelight camera

  argos_lib::RobotInstance
      m_instance;  ///< Contains either the competition bot or practice bot. Differentiates between the two
};
