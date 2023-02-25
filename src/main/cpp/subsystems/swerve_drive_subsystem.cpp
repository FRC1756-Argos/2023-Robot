/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/swerve_drive_subsystem.h"

#include <argos_lib/config/cancoder_config.h>
#include <argos_lib/config/falcon_config.h>
#include <argos_lib/general/angle_utils.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <memory>

#include "Constants.h"
#include "utils/sensor_conversions.h"

using namespace argos_lib::swerve;
using argos_lib::angle::ConstrainAngle;

SwerveDriveSubsystem::SwerveDriveSubsystem(const argos_lib::RobotInstance instance)
    : m_instance(instance)
    , m_controlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl)
    , m_frontLeft(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::frontLeftDrive :
                                                                      address::practice_bot::drive::frontLeftDrive,
                  instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::frontLeftTurn :
                                                                      address::practice_bot::drive::frontLeftTurn,
                  instance == argos_lib::RobotInstance::Competition ? address::comp_bot::encoders::frontLeftEncoder :
                                                                      address::practice_bot::encoders::frontLeftEncoder)
    , m_frontRight(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::frontRightDrive :
                                                                       address::practice_bot::drive::frontRightDrive,
                   instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::frontRightTurn :
                                                                       address::practice_bot::drive::frontRightTurn,
                   instance == argos_lib::RobotInstance::Competition ?
                       address::comp_bot::encoders::frontRightEncoder :
                       address::practice_bot::encoders::frontRightEncoder)
    , m_backRight(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::backRightDrive :
                                                                      address::practice_bot::drive::backRightDrive,
                  instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::backRightTurn :
                                                                      address::practice_bot::drive::backRightTurn,
                  instance == argos_lib::RobotInstance::Competition ? address::comp_bot::encoders::backRightEncoder :
                                                                      address::practice_bot::encoders::backRightEncoder)
    , m_backLeft(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::backLeftDrive :
                                                                     address::practice_bot::drive::backLeftDrive,
                 instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::backLeftTurn :
                                                                     address::practice_bot::drive::backLeftTurn,
                 instance == argos_lib::RobotInstance::Competition ? address::comp_bot::encoders::backLeftEncoder :
                                                                     address::practice_bot::encoders::backLeftEncoder)
    , m_imu(frc::ADIS16448_IMU::kZ, frc::SPI::Port::kMXP, frc::ADIS16448_IMU::CalibrationTime::_8s)
    , m_pigeonIMU(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::sensors::pigeonIMU.address :
                                                                      address::practice_bot::sensors::pigeonIMU.address,
                  std::string(instance == argos_lib::RobotInstance::Competition ?
                                  address::comp_bot::sensors::pigeonIMU.busName :
                                  address::practice_bot::sensors::pigeonIMU.busName))
    , m_swerveDriveKinematics(
          // Forward is positive X, left is positive Y
          // Front Left
          frc::Translation2d(measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontLeftLOffset,
                             measure_up::chassis::width / 2 - measure_up::swerve_offsets::frontLeftWOffset),
          // Front Right
          frc::Translation2d(measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontRightLOffset,
                             -measure_up::chassis::width / 2 + measure_up::swerve_offsets::frontRightWOffset),
          // Back Right
          frc::Translation2d(-measure_up::chassis::length / 2 + measure_up::swerve_offsets::backRightLOffset,
                             -measure_up::chassis::width / 2 + measure_up::swerve_offsets::backRightWOffset),
          // Back Left
          frc::Translation2d(-measure_up::chassis::length / 2 + measure_up::swerve_offsets::backLeftLOffset,
                             measure_up::chassis::width / 2 - measure_up::swerve_offsets::backLeftWOffset))
    , m_odometry{m_swerveDriveKinematics, frc::Rotation2d(GetIMUYaw()), GetCurrentModulePositions(), frc::Pose2d{}}
    , m_prevOdometryAngle{0_deg}
    , m_continuousOdometryOffset{0_deg}
    , m_poseEstimator{m_swerveDriveKinematics, frc::Rotation2d(GetIMUYaw()), GetCurrentModulePositions(), frc::Pose2d{}}
    , m_fsStorage(paths::swerveHomesPath)
    , m_followingProfile(false)
    , m_profileComplete(false)
    , m_pActiveSwerveProfile(nullptr)
    , m_swerveProfileStartTime()
    , m_linearPID(instance == argos_lib::RobotInstance::Competition ?
                      frc2::PIDController{controlLoop::comp_bot::drive::linear_follower::kP,
                                          controlLoop::comp_bot::drive::linear_follower::kI,
                                          controlLoop::comp_bot::drive::linear_follower::kD} :
                      frc2::PIDController{controlLoop::practice_bot::drive::linear_follower::kP,
                                          controlLoop::practice_bot::drive::linear_follower::kI,
                                          controlLoop::practice_bot::drive::linear_follower::kD})
    , m_rotationalPID(instance == argos_lib::RobotInstance::Competition ?
                          frc::ProfiledPIDController<units::radians>{
                              controlLoop::comp_bot::drive::rotational_follower::kP,
                              controlLoop::comp_bot::drive::rotational_follower::kI,
                              controlLoop::comp_bot::drive::rotational_follower::kD,
                              frc::TrapezoidProfile<units::radians>::Constraints(
                                  controlLoop::comp_bot::drive::rotational_follower::angularVelocity,
                                  controlLoop::comp_bot::drive::rotational_follower::angularAcceleration)} :
                          frc::ProfiledPIDController<units::radians>{
                              controlLoop::practice_bot::drive::rotational_follower::kP,
                              controlLoop::practice_bot::drive::rotational_follower::kI,
                              controlLoop::practice_bot::drive::rotational_follower::kD,
                              frc::TrapezoidProfile<units::radians>::Constraints(
                                  controlLoop::practice_bot::drive::rotational_follower::angularVelocity,
                                  controlLoop::practice_bot::drive::rotational_follower::angularAcceleration)})
    , m_followerController{m_linearPID, m_linearPID, m_rotationalPID}
    , m_driveMotorPIDTuner(
          "argos/drive/motors",
          {&m_frontLeft.m_drive, &m_frontRight.m_drive, &m_backRight.m_drive, &m_backLeft.m_drive},
          0,
          argos_lib::ClosedLoopSensorConversions{
              argos_lib::GetSensorConversionFactor(sensor_conversions::swerve_drive::drive::ToDistance),
              argos_lib::GetSensorConversionFactor(sensor_conversions::swerve_drive::drive::ToVelocity),
              argos_lib::GetSensorConversionFactor(sensor_conversions::swerve_drive::drive::ToVelocity)}) {
  // TURN MOTORS CONFIG
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::frontLeftTurn,
                                         motorConfig::practice_bot::drive::frontLeftTurn>(
      m_frontLeft.m_turn, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::frontRightTurn,
                                         motorConfig::practice_bot::drive::frontRightTurn>(
      m_frontRight.m_turn, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::backRightTurn,
                                         motorConfig::practice_bot::drive::backRightTurn>(
      m_backRight.m_turn, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::backLeftTurn,
                                         motorConfig::practice_bot::drive::backLeftTurn>(
      m_backLeft.m_turn, 100_ms, instance);

  // DRIVE MOTOR CONFIGS
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_frontLeft.m_drive, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_frontRight.m_drive, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_backLeft.m_drive, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_backRight.m_drive, 100_ms, instance);

  // CAN ENCODER CONFIG
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::comp_bot::drive::frontLeftTurn,
                                             motorConfig::practice_bot::drive::frontLeftTurn>(
      m_frontLeft.m_encoder, 100_ms, instance);
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::comp_bot::drive::frontRightTurn,
                                             motorConfig::practice_bot::drive::frontRightTurn>(
      m_frontRight.m_encoder, 100_ms, instance);
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::comp_bot::drive::backRightTurn,
                                             motorConfig::practice_bot::drive::backRightTurn>(
      m_backRight.m_encoder, 100_ms, instance);
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::comp_bot::drive::backLeftTurn,
                                             motorConfig::practice_bot::drive::backLeftTurn>(
      m_backLeft.m_encoder, 100_ms, instance);

  InitializeMotors();

  m_rotationalPID.EnableContinuousInput(-180_deg, 180_deg);
}

void SwerveDriveSubsystem::Disable() {
  m_controlMode = DriveControlMode::fieldCentricControl;
  m_followingProfile = false;
  m_profileComplete = false;
  StopDrive();
}

// SWERVE DRIVE SUBSYSTEM MEMBER FUNCTIONS

wpi::array<frc::SwerveModuleState, 4> SwerveDriveSubsystem::GetRawModuleStates(
    SwerveDriveSubsystem::Velocities velocities) {
  // IF SPEEDS ZERO, SET MOTORS TO ZERO AND RETURN
  if (velocities.fwVelocity == 0 && velocities.sideVelocity == 0 && velocities.rotVelocity == 0) {
    StopDrive();
    frc::ChassisSpeeds emptySpeeds{units::make_unit<units::velocity::meters_per_second_t>(0),
                                   units::make_unit<units::velocity::meters_per_second_t>(0),
                                   units::make_unit<units::angular_velocity::radians_per_second_t>(0)};

    return m_swerveDriveKinematics.ToSwerveModuleStates(emptySpeeds);
  }

  switch (m_controlMode) {
    case (DriveControlMode::
              fieldCentricControl): {  // Construct speeds with field-relative speeds and current IMU Z angle.
      frc::ChassisSpeeds fieldCentricSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          units::make_unit<units::meters_per_second_t>(velocities.fwVelocity),
          units::make_unit<units::meters_per_second_t>(velocities.sideVelocity),
          units::make_unit<units::angular_velocity::radians_per_second_t>(velocities.rotVelocity),
          frc::Rotation2d(GetFieldCentricAngle()));

      // Return the speeds to consumer
      return m_swerveDriveKinematics.ToSwerveModuleStates(fieldCentricSpeeds);
    }

    case (DriveControlMode::robotCentricControl): {
      // Construct speeds just the same as in the current main drive function
      frc::ChassisSpeeds robotCentricSpeeds{
          units::make_unit<units::velocity::meters_per_second_t>(velocities.fwVelocity),
          units::make_unit<units::velocity::meters_per_second_t>(velocities.sideVelocity),
          units::make_unit<units::angular_velocity::radians_per_second_t>(velocities.rotVelocity)};

      return m_swerveDriveKinematics.ToSwerveModuleStates(robotCentricSpeeds);
    }
  }
  frc::ChassisSpeeds emptySpeeds{units::make_unit<units::velocity::meters_per_second_t>(0),
                                 units::make_unit<units::velocity::meters_per_second_t>(0),
                                 units::make_unit<units::angular_velocity::radians_per_second_t>(0)};

  return m_swerveDriveKinematics.ToSwerveModuleStates(emptySpeeds);
}

wpi::array<frc::SwerveModuleState, 4> SwerveDriveSubsystem::GetCurrentModuleStates() {
  return {m_frontLeft.GetState(), m_frontRight.GetState(), m_backRight.GetState(), m_backLeft.GetState()};
}

wpi::array<frc::SwerveModulePosition, 4> SwerveDriveSubsystem::GetCurrentModulePositions() {
  return {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backRight.GetPosition(), m_backLeft.GetPosition()};
}

void SwerveDriveSubsystem::SwerveDrive(const double& fwVelocity,
                                       const double& sideVelocity,
                                       const double& rotVelocity) {
  // UpdateOdometry();
  UpdateEstimatedPose();
  if (fwVelocity == 0 && sideVelocity == 0 && rotVelocity == 0) {
    if (!m_followingProfile) {
      StopDrive();
      return;
    }
  } else {
    // Manual override
    m_followingProfile = false;
    m_profileComplete = false;
  }

  SwerveDriveSubsystem::Velocities velocities{fwVelocity, sideVelocity, rotVelocity};

  // DEBUG STUFF
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) fwVelocity", fwVelocity);
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) sideVelocity", sideVelocity);
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) rotVelocity", rotVelocity);
  frc::SmartDashboard::PutNumber("CONTROL MODE", m_controlMode);
  frc::SmartDashboard::PutNumber("IMU ANGLE", m_imu.GetAngle().to<double>());
  frc::SmartDashboard::PutNumber("IMU PIGEON ANGLE", m_pigeonIMU.GetYaw());

  // SET MODULES BASED OFF OF CONTROL MODE
  auto moduleStates = GetCurrentModuleStates();
  frc::Trajectory::State desiredProfileState;
  if (m_followingProfile && m_pActiveSwerveProfile) {
    const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                                   m_swerveProfileStartTime);
    if (!m_pActiveSwerveProfile->IsFinished(elapsedTime)) {
      desiredProfileState = m_pActiveSwerveProfile->Calculate(elapsedTime);
      // const auto controllerChassisSpeeds = m_followerController.Calculate(
      //     m_odometry.GetPose(), desiredProfileState, m_pActiveSwerveProfile->GetEndAngle());

      const auto controllerChassisSpeeds = m_followerController.Calculate(
          m_poseEstimator.GetEstimatedPosition(), desiredProfileState, m_pActiveSwerveProfile->GetEndAngle());
      moduleStates = m_swerveDriveKinematics.ToSwerveModuleStates(controllerChassisSpeeds);
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired X",
                                     units::inch_t{desiredProfileState.pose.X()}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired Y",
                                     units::inch_t{desiredProfileState.pose.Y()}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired Angle",
                                     desiredProfileState.pose.Rotation().Degrees().to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired Curvature",
                                     units::unit_t<units::compound_unit<units::degrees, units::inverse<units::feet>>>{
                                         desiredProfileState.curvature}
                                         .to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) End Angle", m_pActiveSwerveProfile->GetEndAngle().to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired Vel",
                                     units::feet_per_second_t{desiredProfileState.velocity}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Current X",
                                     units::inch_t{GetContinuousOdometry().X()}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Current Y",
                                     units::inch_t{GetContinuousOdometry().Y()}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Current Angle",
                                     GetContinuousOdometry().Rotation().Degrees().to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Controller Vx",
                                     units::feet_per_second_t{controllerChassisSpeeds.vx}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Controller Vy",
                                     units::feet_per_second_t{controllerChassisSpeeds.vy}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Controller Omega",
                                     units::degrees_per_second_t{controllerChassisSpeeds.omega}.to<double>());
      // frc::SmartDashboard::PutNumber("(SwerveFollower) Current Vel", units::feet_per_second_t{desiredProfileState.velocity}.to<double>());
    } else {
      // Finished profile
      m_followingProfile = false;
      m_profileComplete = true;
    }
  } else if (m_followingProfile) {
    // Bad profile
    m_followingProfile = false;
    m_profileComplete = false;
  } else {
    moduleStates = GetRawModuleStates(velocities);
  }

  moduleStates.at(0) = argos_lib::swerve::Optimize(
      moduleStates.at(0),
      sensor_conversions::swerve_drive::turn::ToAngle(m_frontLeft.m_turn.GetSelectedSensorPosition()),
      0_rpm,
      0_fps,
      12_fps);
  moduleStates.at(1) = argos_lib::swerve::Optimize(
      moduleStates.at(1),
      sensor_conversions::swerve_drive::turn::ToAngle(m_frontRight.m_turn.GetSelectedSensorPosition()),
      0_rpm,
      0_fps,
      12_fps);
  moduleStates.at(2) = argos_lib::swerve::Optimize(
      moduleStates.at(2),
      sensor_conversions::swerve_drive::turn::ToAngle(m_backRight.m_turn.GetSelectedSensorPosition()),
      0_rpm,
      0_fps,
      12_fps);
  moduleStates.at(3) = argos_lib::swerve::Optimize(
      moduleStates.at(3),
      sensor_conversions::swerve_drive::turn::ToAngle(m_backLeft.m_turn.GetSelectedSensorPosition()),
      0_rpm,
      0_fps,
      12_fps);

  // Give module state values to motors

  if (m_followingProfile) {
    // When following profile, use closed-loop velocity
    // FRONT LEFT
    m_frontLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                            sensor_conversions::swerve_drive::drive::ToSensorVelocity(
                                moduleStates.at(indexes::swerveModules::frontLeftIndex).speed));

    m_frontLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           sensor_conversions::swerve_drive::turn::ToSensorUnit(
                               moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees()));

    // FRONT RIGHT
    m_frontRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                             sensor_conversions::swerve_drive::drive::ToSensorVelocity(
                                 moduleStates.at(indexes::swerveModules::frontRightIndex).speed));

    m_frontRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                            sensor_conversions::swerve_drive::turn::ToSensorUnit(
                                moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees()));

    // BACK RIGHT
    m_backRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                            sensor_conversions::swerve_drive::drive::ToSensorVelocity(
                                moduleStates.at(indexes::swerveModules::backRightIndex).speed));

    m_backRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           sensor_conversions::swerve_drive::turn::ToSensorUnit(
                               moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees()));

    // BACK LEFT
    m_backLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                           sensor_conversions::swerve_drive::drive::ToSensorVelocity(
                               moduleStates.at(indexes::swerveModules::backLeftIndex).speed));

    m_backLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                          sensor_conversions::swerve_drive::turn::ToSensorUnit(
                              moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees()));
  } else {
    // FRONT LEFT
    m_frontLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
                            moduleStates.at(indexes::swerveModules::frontLeftIndex).speed.to<double>());

    m_frontLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           sensor_conversions::swerve_drive::turn::ToSensorUnit(
                               moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees()));

    // FRONT RIGHT
    m_frontRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
                             moduleStates.at(indexes::swerveModules::frontRightIndex).speed.to<double>());

    m_frontRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                            sensor_conversions::swerve_drive::turn::ToSensorUnit(
                                moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees()));

    // BACK RIGHT
    m_backRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
                            moduleStates.at(indexes::swerveModules::backRightIndex).speed.to<double>());

    m_backRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           sensor_conversions::swerve_drive::turn::ToSensorUnit(
                               moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees()));

    // BACK LEFT
    m_backLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
                           moduleStates.at(indexes::swerveModules::backLeftIndex).speed.to<double>());

    m_backLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                          sensor_conversions::swerve_drive::turn::ToSensorUnit(
                              moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees()));
  }

  // DEBUG STUFF
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) FL speed",
                                 moduleStates.at(indexes::swerveModules::frontLeftIndex).speed.to<double>());
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) FL turn",
                                 moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees().to<double>());

  frc::SmartDashboard::PutNumber("(DRIVETRAIN) FR speed",
                                 moduleStates.at(indexes::swerveModules::frontRightIndex).speed.to<double>());
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) FR turn",
                                 moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees().to<double>());

  frc::SmartDashboard::PutNumber("(DRIVETRAIN) BR speed",
                                 moduleStates.at(indexes::swerveModules::backRightIndex).speed.to<double>());
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) BR turn",
                                 moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees().to<double>());

  frc::SmartDashboard::PutNumber("(DRIVETRAIN) BL speed",
                                 moduleStates.at(indexes::swerveModules::backLeftIndex).speed.to<double>());
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) BL turn",
                                 moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees().to<double>());
}

void SwerveDriveSubsystem::StopDrive() {
  m_frontLeft.m_drive.Set(0.0);
  m_frontLeft.m_turn.Set(0.0);
  m_frontRight.m_drive.Set(0.0);
  m_frontRight.m_turn.Set(0.0);
  m_backRight.m_drive.Set(0.0);
  m_backRight.m_turn.Set(0.0);
  m_backLeft.m_drive.Set(0.0);
  m_backLeft.m_turn.Set(0.0);
}

void SwerveDriveSubsystem::Home(const units::degree_t& angle) {
  HomeToFS(angle);

  // RE-ZERO THE IMU
  ResetIMUYaw();

  // SetPosition expects a value in degrees
  m_frontLeft.m_encoder.SetPosition(angle.to<double>(), 50);
  m_frontRight.m_encoder.SetPosition(angle.to<double>(), 50);
  m_backRight.m_encoder.SetPosition(angle.to<double>(), 50);
  m_backLeft.m_encoder.SetPosition(angle.to<double>(), 50);
}

void SwerveDriveSubsystem::FieldHome(units::degree_t homeAngle, bool updateOdometry) {
  m_fieldHomeOffset = -GetIMUYaw() - homeAngle;
  if (updateOdometry) {
    // Update odometry as well
    const auto currentPose = m_poseEstimator.GetEstimatedPosition();  // m_odometry.GetPose();
    m_poseEstimator.ResetPosition(
        -GetIMUYaw(), GetCurrentModulePositions(), frc::Pose2d{currentPose.Translation(), frc::Rotation2d(homeAngle)});
    m_prevOdometryAngle = m_poseEstimator.GetEstimatedPosition().Rotation().Degrees();
    m_continuousOdometryOffset = 0_deg;
  }
}

void SwerveDriveSubsystem::InitializeOdometry(const frc::Pose2d& currentPose) {
  m_poseEstimator.ResetPosition(-GetIMUYaw(), GetCurrentModulePositions(), currentPose);
  m_prevOdometryAngle = m_poseEstimator.GetEstimatedPosition().Rotation().Degrees();
  m_continuousOdometryOffset = 0_deg;
  // Since we know the position, might as well update the driving orientation as well
  FieldHome(currentPose.Rotation().Degrees(), false);
}

frc::Rotation2d SwerveDriveSubsystem::GetContinuousOdometryAngle() {
  const auto latestOdometry = m_poseEstimator.GetEstimatedPosition();  // m_odometry.GetPose();

  if (m_prevOdometryAngle > 90_deg && latestOdometry.Rotation().Degrees() < -(90_deg)) {
    m_continuousOdometryOffset += 360_deg;
  } else if (m_prevOdometryAngle < -(90_deg) && latestOdometry.Rotation().Degrees() > 90_deg) {
    m_continuousOdometryOffset -= 360_deg;
  }
  m_prevOdometryAngle = latestOdometry.Rotation().Degrees();

  return frc::Rotation2d{latestOdometry.Rotation().Degrees() + m_continuousOdometryOffset};
}

frc::Rotation2d SwerveDriveSubsystem::GetContinuousPoseEstAngle() {
  const auto latestOdometry = m_poseEstimator.GetEstimatedPosition();

  if (m_prevOdometryAngle > 90_deg && latestOdometry.Rotation().Degrees() < -(90_deg)) {
    m_continuousOdometryOffset += 360_deg;
  } else if (m_prevOdometryAngle < -(90_deg) && latestOdometry.Rotation().Degrees() > 90_deg) {
    m_continuousOdometryOffset -= 360_deg;
  }
  m_prevOdometryAngle = latestOdometry.Rotation().Degrees();

  return frc::Rotation2d{latestOdometry.Rotation().Degrees() + m_continuousOdometryOffset};
}

frc::Pose2d SwerveDriveSubsystem::GetContinuousOdometry() {
  const auto discontinuousOdometry = m_poseEstimator.GetEstimatedPosition();  // m_odometry.GetPose();
  return frc::Pose2d{discontinuousOdometry.Translation(), GetContinuousPoseEstAngle()};
}

frc::Pose2d SwerveDriveSubsystem::UpdateOdometry() {
  const auto newPose = m_odometry.Update(frc::Rotation2d{-GetIMUYaw()}, GetCurrentModulePositions());
  const auto continuousOdometryAngle = GetContinuousOdometryAngle();
  frc::SmartDashboard::PutNumber("(Odometry) X", units::inch_t{newPose.X()}.to<double>());
  frc::SmartDashboard::PutNumber("(Odometry) Y", units::inch_t{newPose.Y()}.to<double>());
  frc::SmartDashboard::PutNumber("(Odometry) Angle", newPose.Rotation().Degrees().to<double>());
  frc::SmartDashboard::PutNumber("(Odometry) Continuous Angle", continuousOdometryAngle.Degrees().to<double>());
  return frc::Pose2d{newPose.Translation(), continuousOdometryAngle};
}

frc::Pose2d SwerveDriveSubsystem::UpdateEstimatedPose() {
  const auto newEstPose = m_poseEstimator.Update(frc::Rotation2d{-GetIMUYaw()}, GetCurrentModulePositions());
  const auto continuousEstPoseAngle = GetContinuousPoseEstAngle();
  frc::SmartDashboard::PutNumber("(Odometry) X", units::inch_t{newEstPose.X()}.to<double>());
  frc::SmartDashboard::PutNumber("(Odometry) Y", units::inch_t{newEstPose.Y()}.to<double>());
  frc::SmartDashboard::PutNumber("(Odometry) Angle", newEstPose.Rotation().Degrees().to<double>());
  frc::SmartDashboard::PutNumber("(Odometry) Continuous Angle", GetContinuousPoseEstAngle().Degrees().to<double>());
  return frc::Pose2d{newEstPose.Translation(), continuousEstPoseAngle};
}

units::degree_t SwerveDriveSubsystem::GetFieldCentricAngle() const {
  return -GetIMUYaw() - m_fieldHomeOffset;
}

frc::Pose2d SwerveDriveSubsystem::GetPoseEstimate(const frc::Pose2d& robotPose, const units::millisecond_t& latency) {
  const auto newEstPose = m_poseEstimator.Update(frc::Rotation2d{-GetIMUYaw()}, GetCurrentModulePositions());
  const auto continuousEstPoseAngle = GetContinuousPoseEstAngle();

  // Account for Vision Measurement here
  frc::Timer timer;
  const auto timeStamp = timer.GetFPGATimestamp() - latency;
  m_poseEstimator.AddVisionMeasurement(robotPose, timeStamp);

  frc::SmartDashboard::PutNumber("(Est Pose) X", units::inch_t{newEstPose.X()}.to<double>());
  frc::SmartDashboard::PutNumber("(Est Pose) Y", units::inch_t{newEstPose.Y()}.to<double>());
  frc::SmartDashboard::PutNumber("(Est Pose) Angle", newEstPose.Rotation().Degrees().to<double>());
  frc::SmartDashboard::PutNumber("(Est Pose) Continuous Angle", continuousEstPoseAngle.Degrees().to<double>());

  frc::SmartDashboard::PutNumber("(Est Pose After Vision) X",
                                 units::inch_t{m_poseEstimator.GetEstimatedPosition().X()}.to<double>());
  frc::SmartDashboard::PutNumber("(Est Pose After Vision) Y",
                                 units::inch_t{m_poseEstimator.GetEstimatedPosition().Y()}.to<double>());
  frc::SmartDashboard::PutNumber("(Est Pose After Vision) Angle",
                                 m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().to<double>());
  frc::SmartDashboard::PutNumber("(Est Pose After Vision) Continuous Angle",
                                 GetContinuousPoseEstAngle().Degrees().to<double>());

  return frc::Pose2d{m_poseEstimator.GetEstimatedPosition().Translation(), GetContinuousPoseEstAngle()};
  // return GetContinuousOdometry();
}

void SwerveDriveSubsystem::SetControlMode(SwerveDriveSubsystem::DriveControlMode controlMode) {
  m_controlMode = controlMode;
}

void SwerveDriveSubsystem::InitializeMotors() {
  InitializeMotorsFromFS();
}

void SwerveDriveSubsystem::HomeToFS(const units::degree_t& angle) {
  const argos_lib::swerve::SwerveModulePositions homes{
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_frontLeft.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg),
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_frontRight.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg),
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_backRight.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg),
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_backLeft.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg)};

  m_fsStorage.Save(homes);
}

void SwerveDriveSubsystem::InitializeMotorsFromFS() {
  std::optional<argos_lib::swerve::SwerveModulePositions> homes = m_fsStorage.Load();

  if (!homes) {
    // ALERT HERE THAT THERE ARE NO VALUES, BUT FOR NOW, JUST PRINT
    std::printf("%d HEY NO SAVED VALUES IN std::FILE SYSTEM!!!!", __LINE__);
    return;
  }

  // GET CURRENT VALUES
  units::degree_t frontLeft_current = units::make_unit<units::degree_t>(m_frontLeft.m_encoder.GetAbsolutePosition());
  units::degree_t frontRight_current = units::make_unit<units::degree_t>(m_frontRight.m_encoder.GetAbsolutePosition());
  units::degree_t backRight_current = units::make_unit<units::degree_t>(m_backRight.m_encoder.GetAbsolutePosition());
  units::degree_t backLeft_current = units::make_unit<units::degree_t>(m_backLeft.m_encoder.GetAbsolutePosition());

  // SUBTRACT SAVED FROM CURRENT
  const units::degree_t frontLeftCalibrated = frontLeft_current - homes.value().FrontLeft;
  const units::degree_t frontRightCalibrated = frontRight_current - homes.value().FrontRight;
  const units::degree_t backRightCalibrated = backRight_current - homes.value().RearRight;
  const units::degree_t backLeftCalibrated = backLeft_current - homes.value().RearLeft;

  // ASSIGN DIFFERENCE TO CURRENT MOTOR RELATIVE POSITION
  m_frontLeft.m_encoder.SetPosition(frontLeftCalibrated.to<double>(), 50);
  m_frontRight.m_encoder.SetPosition(frontRightCalibrated.to<double>(), 50);
  m_backRight.m_encoder.SetPosition(backRightCalibrated.to<double>(), 50);
  m_backLeft.m_encoder.SetPosition(backLeftCalibrated.to<double>(), 50);
}

// SWERVE MODULE SUBSYSTEM FUNCTIONS
SwerveModule::SwerveModule(const argos_lib::CANAddress& driveAddr,
                           const argos_lib::CANAddress& turnAddr,
                           const argos_lib::CANAddress& encoderAddr)
    : m_drive(driveAddr.address, std::string(driveAddr.busName))
    , m_turn(turnAddr.address, std::string(turnAddr.busName))
    , m_encoder(encoderAddr.address, std::string(encoderAddr.busName)) {}

frc::SwerveModuleState SwerveModule::GetState() {
  return frc::SwerveModuleState{
      sensor_conversions::swerve_drive::drive::ToVelocity(m_drive.GetSelectedSensorVelocity()),
      frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(m_turn.GetSelectedSensorPosition())}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return frc::SwerveModulePosition{
      sensor_conversions::swerve_drive::drive::ToDistance(m_drive.GetSelectedSensorPosition()),
      frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(m_turn.GetSelectedSensorPosition())}};
}

void SwerveDriveSubsystem::UpdateFollowerLinearPIDParams(double kP, double kI, double kD) {
  m_linearPID.SetPID(kP, kI, kD);
  m_linearPID.Reset();
}

void SwerveDriveSubsystem::UpdateFollowerRotationalPIDParams(double kP, double kI, double kD) {
  m_rotationalPID.SetPID(kP, kI, kD);
  m_rotationalPID.Reset(m_rotationalPID.GetGoal());
}

void SwerveDriveSubsystem::UpdateFollowerRotationalPIDConstraints(
    frc::TrapezoidProfile<units::degrees>::Constraints constraints) {
  m_rotationalPID.SetConstraints(
      frc::TrapezoidProfile<units::radians>::Constraints{constraints.maxVelocity, constraints.maxAcceleration});
  m_rotationalPID.Reset(m_rotationalPID.GetGoal());
}

void SwerveDriveSubsystem::StartDrivingProfile(SwerveTrapezoidalProfileSegment newProfile) {
  m_profileComplete = false;
  m_pActiveSwerveProfile = std::make_unique<SwerveTrapezoidalProfileSegment>(newProfile);
  m_followerController = frc::HolonomicDriveController(m_linearPID, m_linearPID, m_rotationalPID);
  m_swerveProfileStartTime = std::chrono::steady_clock::now();
  m_followingProfile = true;
}

void SwerveDriveSubsystem::CancelDrivingProfile() {
  m_profileComplete = false;
  m_followingProfile = false;
}

bool SwerveDriveSubsystem::ProfileIsComplete() const {
  return m_profileComplete;
}

units::degree_t SwerveDriveSubsystem::GetIMUYaw() const {
  return -units::degree_t{m_pigeonIMU.GetYaw()};
}

void SwerveDriveSubsystem::ResetIMUYaw() {
  m_pigeonIMU.SetYaw(0);
}

frc::ChassisSpeeds SwerveDriveSubsystem::GetChassisVelocity() {
  return m_swerveDriveKinematics.ToChassisSpeeds(GetCurrentModuleStates());
}
