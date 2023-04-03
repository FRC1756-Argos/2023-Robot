/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/filter/LinearFilter.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc2/command/SubsystemBase.h>

#include "argos_lib/config/config_types.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "swerve_drive_subsystem.h"

class LimelightTarget {
 private:
  frc::Pose3d m_robotPose;              ///< 3d pose of robot relative to field center
  frc::Pose3d m_robotPoseWPI;           ///< 3d pose of robot relative to WPI reference for active alliance
  frc::Pose3d m_robotPoseTagSpace;      ///< 3d pose of robot relative to primary april tag (biggest?)
  bool m_hasTargets;                    ///< True if the camera has a target it can read
  units::degree_t m_pitch;              ///< Pitch of target relative to camera -24.85 to 24.85 degrees
  units::degree_t m_yaw;                ///< Yaw of target relative to camera -31.65 to 31.65 degrees
  double m_area;                        ///< Area of the target in percentage of total pixels
  units::millisecond_t m_totalLatency;  ///< Total latency
  int m_tid;                            ///< Id of the primary april tag in view
  frc::LinearFilter<units::degree_t> m_txFilter;
  frc::LinearFilter<units::degree_t> m_tyFilter;
  bool m_resetFilterFlag;

 public:
  LimelightTarget()
      : m_txFilter{frc::LinearFilter<units::degree_t>::SinglePoleIIR(0.01, 0.02_s)}
      , m_tyFilter{frc::LinearFilter<units::degree_t>::SinglePoleIIR(0.01, 0.02_s)}
      , m_resetFilterFlag{false} {}

  /**
   * @brief Wraps members of LimelightTarget for use elsewhere
   *
   */
  struct tValues {
    frc::Pose3d robotPose;              ///< @copydoc LimelightTarget::m_robotPose
    frc::Pose3d robotPoseWPI;           ///< @copydoc LimelightTarget::m_robotPoseWPI
    frc::Pose3d robotPoseTagSpace;      ///< @copydoc LimelightTarget::m_robotPoseTagSpace
    bool hasTargets;                    ///< @copydoc LimelightTarget::m_hasTargets
    units::degree_t m_pitch;            ///< @copydoc LimelightTarget::m_pitch
    units::degree_t m_yaw;              ///< @copydoc LimelightTarget::m_yaw
    double m_area;                      ///< @copydoc LimelightTarget::m_area
    units::millisecond_t totalLatency;  ///< @copydoc LimelightTarget::m_totalLatency
    int tid;                            ///< @copydoc LimelightTarget::m_tid
  };

  /**
   * @brief Get the values of the camera's current target
   *
   * @return tValues
   */
  tValues GetTarget(bool filter);

  /**
   * @brief Does the camera see a target?
   *
   * @return true - The camera does see a target
   * @return false - The camera does not see a target
   */
  bool HasTarget();

  void ResetFilters();

  void ResetOnNextTarget();
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

  void RequestTargetFilterReset();
};

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem(const argos_lib::RobotInstance instance, SwerveDriveSubsystem* pDriveSubsystem);

  /**
   * @brief Get the offset to the center of the target
   *
   * @param target Target to aim toward
   * @return Desired offset.
   */
  std::optional<units::degree_t> GetOffsetToTarget(LimelightTarget::tValues target);

  /**
   * @brief Get the robot poses and latencies
   *
   * @return LimelightTarget::tValues
   */
  LimelightTarget::tValues GetCameraTargetValues();

  /**
   * @brief Get the old robot poses and latencies
   *
   * @return LimelightTarget::tValues
   */
  LimelightTarget::tValues m_oldTargetValues;

  /**
   * @brief Get the current offset to the retroreflective tape
   *
   * @return units::degree_t
   */
  std::optional<units::degree_t> GetHorizontalOffsetToTarget();

  /**
   * @brief Get the longitudinal distance to the retroreflective tape
   * @note For now assume we are always seeing the lower pole
   *
   * @return units::inch_t
   */
  std::optional<units::inch_t> GetDistanceToPoleTape();

  void SetReflectiveVisionMode(bool mode);

  bool AimToPlaceCone();

  void RequestFilterReset();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /// @brief it disables (duh)
  void Disable();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  CameraInterface m_cameraInterface;  ///< Interface to limelight camera

  argos_lib::RobotInstance
      m_instance;  ///< Contains either the competition bot or practice bot. Differentiates between the two
  SwerveDriveSubsystem* m_pDriveSubsystem;  ///< Pointer to drivetrain for reading some odometry
};
