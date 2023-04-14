/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/ADIS16448_IMU.h>
#include <frc/Timer.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>

#include "argos_lib/config/config_types.h"
#include "argos_lib/general/nt_motor_pid_tuner.h"
#include "argos_lib/general/nt_subscriber.h"
#include "argos_lib/homing/fs_homing.h"
#include "ctre/Phoenix.h"
#include "frc/StateSpaceUtil.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "utils/swerve_trapezoidal_profile.h"
#include "utils/swerve_trapezoidal_spline.h"

class SwerveModule {
 public:
  // MOTORS
  WPI_TalonFX m_drive;
  WPI_TalonFX m_turn;
  // ENCODER
  CANCoder m_encoder;

  /**
   * @brief Construct a new Swerve Module object
   *
   * @param driveAddr address of the drive motor on the module
   * @param turnAddr address of the turn motor on the module
   * @param encoderAddr address of the encoder on this module
   */
  SwerveModule(const argos_lib::CANAddress& driveAddr,
               const argos_lib::CANAddress& turnAddr,
               const argos_lib::CANAddress& encoderAddr);

  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();
};

/**
 * @brief Subsystem for controlling the swerve drivetrain of the robot
 *
 */
class SwerveDriveSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * @brief Enumerator for either field-centric or robot centric drive modes.
   *
   */
  enum DriveControlMode { fieldCentricControl, robotCentricControl };

  explicit SwerveDriveSubsystem(const argos_lib::RobotInstance instance);

  /**
   * @brief Handle the robot disabling
   */
  void Disable();

  /**
   * @brief Main drive function for the robot
   *
   * @param fwVelocity Percent speed forward.  Range [-1.0, 1.0] where positive 1.0 is full speed forward
   * @param sideVelocity Percent speed perpendicular to the robots front.  Range [-1.0, 1.0] where positive 1.0 is full speed left
   * @param rotVelocity Percent speed of rotation of the chassis.  Range [-1.0, 1.0] where positive 1.0 is full speed counterclockwise
   */
  void SwerveDrive(const double fwVelocity, const double sideVelocity, const double rotVelocity);

  /// @brief Same as polar swerve drive function, but also takes in a rotational velocity to apply ONLY USE IN FIELD-CENTRIC
  /// @param velAngle Angle of velocity vector, [0, 360] with 0 degrees being field-centric home
  /// @param velocity Magnitude of velocity on [0, 1] to apply
  /// @param rotVelocity Percent speed of rotation of the chassis.  Range [-1.0, 1.0] where positive 1.0 is full speed counterclockwise
  void SwerveDrive(const units::degree_t& velAngle, const double& velocity, const double& rotVelocity);

  /// @brief Takes in speeds as a polar vector, and calculates the forward and side velocity to apply ONLY USE IN FIELD-CENTRIC
  /// @param velAngle Angle of velocity vector, [0, 360] with 0 degrees being field-centric home
  /// @param velocity Magnitude of velocity on [0, 1] to apply
  void SwerveDrive(const units::degree_t& velAngle, const double& velocity);

  /**
   * @brief Stop all motors
   */
  void StopDrive();

  /**
   * @brief Save homes to persistent storage and updates module motors
   *
   * @param angle Current angle of all the swerve modules.  They should all be oriented the same during homing.
   *
   */
  void Home(const units::degree_t& angle);

  /**
   * @brief Tell the robot it's in it's correct field-oriented "Front"
   *
   * @param homeAngle Current orientation of the robot
   * @param updateOdometry Also update odometry field-centric angle
   */
  void FieldHome(units::degree_t homeAngle = 0_deg, bool updateOdometry = true);

  /**
   * @brief Set current robot position.  Useful for initializing initial position before autonomous
   *
   * @param currentPose Field-centric pose of the robot
   */
  void InitializeOdometry(const frc::Pose2d& currentPose);

  frc::Rotation2d GetContinuousOdometryAngle();

  frc::Rotation2d GetNearestSquareAngle();

  frc::Pose2d GetContinuousOdometry();

  /**
   * @brief Reads module states & gyro, updates pose estimator, and returns latest pose estimate
   *
   * @return Estimate of robot pose
   */
  frc::Pose2d UpdateEstimatedPose();

  /**
   * @brief Get the field-centric angle of the robot based on gyro and saved reference orientation
   *
   * @return Field-centric angle of the robot where 0 degrees is intake oriented toward
   *         opposing alliance operator station positive CCW.
   */
  units::degree_t GetFieldCentricAngle() const;

  /**
   * @brief Get the latest pose estimate
   *
   * @return Latest pose
   */
  frc::Pose2d GetPoseEstimate(const frc::Pose2d& robotPose, const units::millisecond_t& latency);

  void SetControlMode(SwerveDriveSubsystem::DriveControlMode controlMode);

  /**
   * @brief Initialize motors from persistent storage
   *
   */
  void InitializeMotors();

  /**
   * @brief Change PID parameters for linear follower.  These adjust velocities based on distance
   *        error from path goal
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  void UpdateFollowerLinearPIDParams(double kP, double kI, double kD);

  /**
   * @brief Change PID parameters for rotational follower.  These adjust velocities based on angle
   *        error from path goal
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  void UpdateFollowerRotationalPIDParams(double kP, double kI, double kD);

  /**
   * @brief Update constraints to rotate robot along profiled path
   *
   * @param constraints Rotational velocity and acceleration constraints
   */
  void UpdateFollowerRotationalPIDConstraints(frc::TrapezoidProfile<units::radians>::Constraints constraints);
  void UpdateFollowerRotationalPIDConstraints(frc::TrapezoidProfile<units::degrees>::Constraints constraints);

  /**
   * @brief Start driving a new profile.  This also resets the finished flag
   *
   * @param newProfile Profile to follow with t=0 being the time this function is called
   */
  void StartDrivingProfile(SwerveTrapezoidalProfileSegment newProfile);

  /**
   * @brief Start driving a new profile.  This also resets the finished flag
   *
   * @param newProfile Profile to follow with t=0 being the time this function is called
   */
  void StartDrivingProfile(SwerveTrapezoidalSpline newProfile);

  /**
   * @brief Cancel the current driving profile without marking it complete
   */
  void CancelDrivingProfile();

  /**
   * @brief Check if a driving profile path has been completed
   *
   * @return true when a path is completed and not canceled
   */
  bool ProfileIsComplete() const;

  /**
   * @brief Check if drivetrain is following a profile
   *
   * @return true when robot is following profile
   */
  bool IsFollowingProfile() const;

  /**
   * @brief Get the robot velocity in chassis frame (x toward intake, y toward left) based on
   *        GetCurrentModuleStates() output
   *
   * @return frc::ChassisSpeeds Velocity based on module states
   */
  frc::ChassisSpeeds GetChassisVelocity();

  /**
   * @brief Put the robot wheels in an x shape where it locks the movement of it
   */
  void LockWheels();

  bool GetManualOverride();

  /**
   * @brief Get the robot pitch as determined by the pigeon IMU
   *
   * @return pitch in unit degrees
   */
  units::degree_t GetRobotPitch() const { return units::degree_t{m_pigeonIMU.GetRoll()}; }

  /**
   * @brief Get the rate of robot pitch
   *
   * @return pitch rate in unit degrees per second
   */
  units::degrees_per_second_t GetRobotPitchRate();

 private:
  argos_lib::RobotInstance m_instance;

  DriveControlMode m_controlMode;  ///< Active control mode

  SwerveModule m_frontLeft;   ///< Front left swerve module
  SwerveModule m_frontRight;  ///< Front right swerve module
  SwerveModule m_backRight;   ///< Back right swerve module
  SwerveModule m_backLeft;    ///< Back left swerve module

  // GYROSCOPIC SENSORS
  frc::ADIS16448_IMU m_imu;
  Pigeon2 m_pigeonIMU;

  units::degree_t m_fieldHomeOffset;  ///< Offset from IMU angle to 0 field angle (intake away from driver station)

  /**
 * @brief A struct for holding the 3 different input velocities, for organization
 *
 */
  struct Velocities {
    const double fwVelocity;
    const double sideVelocity;
    const double rotVelocity;
  };

  frc::SwerveDriveKinematics<4> m_swerveDriveKinematics;  ///< Kinematics model for swerve drive system

  frc::SwerveDriveOdometry<4> m_odometry;      ///< Odometry to track robot
  units::degree_t m_prevOdometryAngle;         ///< Last odometry angle used for continuous calculations
  units::degree_t m_continuousOdometryOffset;  ///< Offset to convert [-180,180] odometry angle to continuous angle

  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;  ///< accounts vision-based measurements for odometry

  // std::FILE SYSTEM HOMING STORAGE
  argos_lib::SwerveFSHomingStorage m_fsStorage;  ///< Roborio filesystem access for homes

  bool m_followingProfile;  ///< True when an incomplete drive profile is being run
  bool m_profileComplete;   ///< True once a drive profile has been completed
  bool m_manualOverride;
  std::unique_ptr<SwerveTrapezoidalProfileSegment> m_pActiveSwerveProfile;      ///< Profile to execute
  std::unique_ptr<SwerveTrapezoidalSpline> m_pActiveSwerveSplineProfile;        ///< Profile to execute
  std::chrono::time_point<std::chrono::steady_clock> m_swerveProfileStartTime;  ///< Time when active profile began
  frc::ProfiledPIDController<units::radians>::Constraints m_rotationalPIDConstraints;
  frc2::PIDController m_linearPID;  ///< Correction parameters for x/y error when following drive profile
  frc::HolonomicDriveController m_followerController;  ///< Controller to follow drive profile

  argos_lib::NTMotorPIDTuner m_driveMotorPIDTuner;  ///< Utility to tune drive motors
  argos_lib::NTSubscriber m_linearFollowerTuner_P;
  argos_lib::NTSubscriber m_linearFollowerTuner_I;
  argos_lib::NTSubscriber m_linearFollowerTuner_D;
  argos_lib::NTSubscriber m_rotationalFollowerTuner_P;
  argos_lib::NTSubscriber m_rotationalFollowerTuner_I;
  argos_lib::NTSubscriber m_rotationalFollowerTuner_D;
  argos_lib::NTSubscriber m_rotationalFollowerConstraintTuner_vel;
  argos_lib::NTSubscriber m_rotationalFollowerConstraintTuner_accel;

  /**
 * @brief Get the Raw Module States object and switch between robot-centric and field-centric
 *
 * @param velocities Desired velocity
 * @return wpi::array<frc::SwerveModuleState, 4>
 */
  wpi::array<frc::SwerveModuleState, 4> GetRawModuleStates(SwerveDriveSubsystem::Velocities velocities);

  /**
   * @brief Get the active states of all swerve modules
   *
   * @return Active module states
   */
  wpi::array<frc::SwerveModuleState, 4> GetCurrentModuleStates();

  /**
   * @brief Get the active positions of all swerve modules
   *
   * @return Active module positions
   */
  wpi::array<frc::SwerveModulePosition, 4> GetCurrentModulePositions();

  /**
   * @brief Save homes to a file
   *
   * @param angle Current angle of all the swerve modules.  They should all be oriented the same during homing.
   *
   */
  void HomeToFS(const units::degree_t& angle);

  /**
   * @brief Initialize motors from saved file
   *
   */
  void InitializeMotorsFromFS();

  units::degree_t GetIMUYaw() const;
  void ResetIMUYaw();
};
