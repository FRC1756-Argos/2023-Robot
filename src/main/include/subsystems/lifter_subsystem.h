/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <ctre/Phoenix.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/SubsystemBase.h>

#include <string>

#include "argos_lib/general/log.h"
#include "argos_lib/general/nt_motor_pid_tuner.h"
#include "argos_lib/homing/fs_homing.h"
#include "constants/interpolation_maps.h"
#include "utils/lifter_kinematics.h"

/* —————————————————————————— SUBSYSTEM CLASS —————————————————————————— */

enum class WristPosition { RollersDown, RollersUp, Unknown };

std::string ToString(WristPosition position);

class LifterSubsystem : public frc2::SubsystemBase {
 public:
  struct LifterPosition {
    units::degree_t wristAngle;
    ArmState state;
  };
  explicit LifterSubsystem(argos_lib::RobotInstance instance);

  /// @brief Sets the arm speed
  /// @param speed double, on the interval [-1, 1]
  void SetShoulderSpeed(double speed);

  /// @brief Sets the arm extension motor speed
  /// @param speed double, on the interval [-1, 1]
  void SetArmExtensionSpeed(double speed);

  /// @brief Sets the extension of the arm to "extension" inches
  /// @param extension The inches to extend, from fulcrum to back face of wrist gear
  void SetArmExtension(units::inch_t extension);

  /// @brief Checks if the arm extension is within an error of extension
  /// @param extension The extension of the arm to compare to
  /// @return True if within acceptable error, False otherwise
  bool IsExtensionAt(units::inch_t extension);

  /// @brief Sets the wrist speed
  /// @param speed double, on the interval [-1, 1]
  void SetWristSpeed(double speed);

  /// @brief Sets the shoulder motor to zero
  void StopShoulder();

  /// @brief Sets the arm extension motor to zero
  void StopArmExtension();

  /// @brief Sets the wrist speed to zero
  void StopWrist();

  /// @brief Checks if manual shoulder override member is true
  /// @return True -> Shoulder is manually overridden False -> Shoulder is not overridden
  bool IsShoulderManualOverride();

  /// @brief Sets the override state of the shoulder
  void SetShoulderManualOverride(bool overrideState);

  /// @brief Checks if manual extension override member is true
  /// @return True -> Extension is manually overridden False -> Extension is not overriden
  bool IsExtensionManualOverride();

  /// @brief Sets the override state of the extension
  void SetExtensionManualOverride(bool overrideState);

  /// @brief Checks if manual wrist override member is true
  /// @return True -> Wrist is manually overridden False -> Wrist is not overriden
  bool IsWristManualOverride();

  /// @brief Sets the override state of the wrist
  void SetWristManualOverride(bool overrideState);

  /// @brief Initializes wrist homes from file system
  void InitializeWristHomes();

  /// @brief Sets the wrist angle from min and max angles defined in measure_up. Positive is CW looking down the arm from the fulcrum
  /// @param wristAngle
  void SetWristAngle(units::degree_t wristAngle);

  units::degree_t GetWristAbsoluteAngle();
  double GetLastWristTimestamp();

  /// @brief updating wrist homes for encoder
  void UpdateWristHome();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /// @brief Handle robot disabling
  void Disable();

  /// @brief Detect if arm is in motion
  /// @return True when arm is in motion
  bool IsArmExtensionMoving();

  /// @brief Detect if shoulder is in motion
  /// @return True when shoulder is in motion
  bool IsShoulderMoving();

  /// @brief Checks if home file for arm extension exists
  /// @return True -> File exists, False otherwise
  bool ArmExtensionHomeFileExists();

  /// @brief Update arm home position
  void UpdateArmExtensionHome();

  /// @brief Initializes the arm extension home from file system
  void InitializeArmExtensionHome();

  /// @brief Initializes the homed shoulder value from FS
  void InitializeShoulderHome();

  /// @brief Updates shoulder home in FS, resets relative position on sensor
  void UpdateShoulderHome();

  /// @brief Sets the shoulder joint's angle (see GetShoulderAngle or docs for convention)
  /// @param angle The target angle the shoulder should go to, in an units::degree_t
  void SetShoulderAngle(units::degree_t angle);

  /// @brief Checks if the shoulder is within an error of extension
  /// @param angle The angle of the shoulder to compare to
  /// @return True if within acceptable error, False otherwise
  bool IsShoulderAt(units::degree_t angle);

  /// @brief Uses the kinematics object to calculate robot pose
  /// @return The arm's current pose in robot space as a Translation2d
  frc::Translation2d GetEffectorPose(const WristPosition wristPosition);

  frc::Translation2d GetArmPose();

  /// @brief Uses the kinematics object to calculate joint positions for a given pose, then sets the system to that pose
  /// @param desPose The point in robot space to go to (y is actually z)
  /// @param effectorPosition Wether or not the effector is inverted
  /// @return A LifterPosition representing the state of the different joints
  LifterPosition SetEffectorPose(const frc::Translation2d& desPose, WristPosition effectorPosition);

  LifterPosition SetLifterPose(const frc::Translation2d& desPose);

  /// @brief Is the arm extension homed?
  /// @return True -> Arm extension is homed False -> Arm extension did not home
  bool IsArmExtensionHomed();

  /// @brief Gets the wrist angle (right handle rule with axis towards front of robot)
  /// @return Angle in units::degree_t representing angle of effector
  units::degree_t GetWristAngle();

  WristPosition GetWristPosition();

  /// @brief Gets the arm extension (positive is extend out)
  /// @return Extension of arm from shoulder rotation center as an units::inch_t
  units::inch_t GetArmExtension();

  /// @brief Gets the shoulder's angle with positive being rotating down into the robot, and 0 being strait out behind the robot
  /// @return The shoulder's current angle as an units::degree_t
  units::degree_t GetShoulderAngle();

  units::degree_t GetShoulderBoomAngle();

  /// @brief Gets the lifter's current shoulder position represented as a collection of joint states
  /// @return A LifterPosition containing the state of the shoulder system's joints
  LifterPosition GetLifterPosition();

  /// @brief Converts from a point in 2d space to an arm state
  /// @param pose Point in robot x/z plane where the end effector should be (y is actually z) in Translation2d
  /// @param effectorPosition True -> Effector is inverted False -> Effector is NOT inverted
  /// @return ArmState holding joint properties to get to "pose" point in 2d space
  ArmState ConvertEffectorPose(const frc::Translation2d& pose, WristPosition effectorPosition) const;

  ArmState ConvertLifterPose(const frc::Translation2d& pose) const;

  frc::Translation2d ConvertStateToEffectorPose(const ArmState& state, WristPosition effectorPosition) const;
  frc::Translation2d ConvertStateToLifterPose(const ArmState& state) const;

  units::inch_t ConvertShoulderAngle(units::degree_t angle) const;
  units::inches_per_second_t ConvertShoulderVelocity(units::degree_t angle,
                                                     units::degrees_per_second_t angularVelocity) const;

  bool IsShoulderMPComplete();
  bool IsExtensionMPComplete();
  bool IsWristMPComplete();

  ctre::phoenix::motion::BufferedTrajectoryPointStream& GetShoulderMPStream();
  ctre::phoenix::motion::BufferedTrajectoryPointStream& GetExtensionMPStream();
  ctre::phoenix::motion::BufferedTrajectoryPointStream& GetWristMPStream();

  void StopMotionProfile();
  void StartMotionProfile(size_t shoulderStreamSize, size_t extensionStreamSize, size_t wristStreamSize);

  void ResetPathFaults();
  bool IsFatalPathFault();

  /// @brief Turn off soft limits for arm extension
  void DisableArmExtensionSoftLimits();

  /// @brief Turn on soft limits for arm extension
  void EnableArmExtensionSoftLimits();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Shoulder motors are attached in parallel mechanically to operate shoulder, back motor follows front motor
  WPI_TalonFX m_shoulderDrive;      ///< Shoulder motor closest to front of robot
  WPI_TalonFX m_armExtensionMotor;  ///< Motor that controls extension of arm
  WPI_TalonFX m_wrist;              ///< Motor that controls wrist movement
  CANCoder m_shoulderEncoder;       ///< Encoder that measures shoulder position
  CANCoder m_wristEncoder;          ///< Encoder for measuring wrist position
  CANCoder m_extensionEncoder;      ///< Encoder for measuring extension of arm
  LifterKinematics m_kinematics;    ///< Kinematic model for solving arm joints & position
  argos_lib::ArgosLogger m_logger;  ///< Handles logging errors & info
  argos_lib::FSHomingStorage<units::degree_t> m_shoulderHomeStorage;     ///< File system homing for shoulder
  argos_lib::FSHomingStorage<units::degree_t> m_wristHomingStorage;      ///< File system homing for wrist
  argos_lib::FSHomingStorage<units::degree_t> m_extensionHomingStorage;  ///< File system homing for wrist
  argos_lib::NTMotorPIDTuner m_extensionTuner;  ///< TEMP network tables PID tuner for tuning extension
  argos_lib::NTMotorPIDTuner m_wristTuner;      ///< TEMP network tables PID tuner for tuning wrist
  argos_lib::NTMotorPIDTuner m_shoulderTuner;   ///< TEMP network tables PID tuner for tuning shoulder
  bool m_shoulderHomed;                         ///< True if the shoulder is homed successfully from file system
  bool m_extensionHomed;                        ///< True if extension is homed successfully
  bool m_wristHomed;                            ///< True if wrist was homed successfully from file system
  bool m_shoulderManualOverride;   ///< True if manual control is currently in use, do not execute any closed loop
  bool m_extensionManualOverride;  ///< True if manual control is currently in use, do not execute any closed loop
  bool m_wristManualOverride;      ///< True if manual control is currently in use, do not execute any closed loop

  ctre::phoenix::motion::BufferedTrajectoryPointStream m_shoulderStream;
  ctre::phoenix::motion::BufferedTrajectoryPointStream m_extensionStream;
  ctre::phoenix::motion::BufferedTrajectoryPointStream m_wristStream;

  /// @brief Turn on soft limits for wrist
  void EnableWristSoftLimits();

  /// @brief Turn off soft limits for wrist
  void DisableWristSoftLimits();

  /// @brief Turn on soft limits for arm shoulder
  void EnableShoulderSoftLimits();

  /// @brief Turn off soft limits for arm shoulder
  void DisableShoulderSoftLimits();
};
