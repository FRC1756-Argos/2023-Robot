/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/falcon_config.h>
#include <argos_lib/general/nt_motor_pid_tuner.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

enum class BashGuardPosition { Retracted, Deployed, Stationary };

class BashGuardSubsystem : public frc2::SubsystemBase {
 public:
  explicit BashGuardSubsystem(argos_lib::RobotInstance instance);

  /// @brief Sets bash guard extension speed
  /// @param speed double, on the interval [-1, 1]
  void SetExtensionSpeed(double speed);

  void SetExtensionLength(units::inch_t length);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void UpdateBashGuardHome();

  /// @warning If homing failed, this could still return true
  bool IsBashGuardHomed();

  bool IsBashGuardManualOverride();

  void SetBashGuardManualOverride(bool overrideState);

  units::inch_t GetBashGuardExtension();

  bool IsBashGuardMoving();

  bool IsBashGuardMPComplete();

  void SetHomeFailed(bool failed);

  bool GetHomeFailed();

  ctre::phoenix::motion::BufferedTrajectoryPointStream& GetMPStream();

  void StopMotionProfile();

  void StartMotionProfile(size_t streamSize);

  void Disable();

  void Stop();

  int GetMotorMPBufferCount();

  units::inch_t DecomposeBashExtension(const BashGuardPosition& positionEnum);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_bashGuard;
  argos_lib::NTMotorPIDTuner m_bashTuner;
  bool m_bashGuardManualOverride;
  bool m_bashGuardHomed;
  bool m_bashHomeFailed;

  ctre::phoenix::motion::BufferedTrajectoryPointStream m_bashStream;

  void EnableBashGuardSoftLimits();
  void DisableBashGuardSoftLimits();
};
