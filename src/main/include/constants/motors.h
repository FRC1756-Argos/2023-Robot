/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "addresses.h"
#include "argos_lib/config/status_frame_config.h"
#include "control_loops.h"
#include "ctre/Phoenix.h"
#include "units/current.h"
#include "units/time.h"
#include "units/voltage.h"

namespace motorConfig {
  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configuration settings shared by all robot configurations
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace common {
    constexpr static auto neutralDeadband = 0.001;
    constexpr static auto voltCompSat = 11.0_V;
  }  // namespace common

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configurations specific to competition robot
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace comp_bot {
    namespace drive {
      struct genericDrive {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::drive::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::drive::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::drive::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::drive::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::drive::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::drive::allowableError;
      };
      struct frontLeftTurn {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::frontLeftEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::rotate::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::rotate::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::rotate::allowableError;
      };
      struct frontRightTurn {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::frontRightEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::rotate::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::rotate::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::rotate::allowableError;
      };
      struct backRightTurn {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::backRightEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::rotate::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::rotate::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::rotate::allowableError;
      };
      struct backLeftTurn {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::backLeftEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::rotate::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::rotate::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::rotate::allowableError;
      };

    }  // namespace drive
    namespace lifter {
      // TODO both of these are temporarily set to generic drive motor configs
      //  but will need their own configs and control loop values in the future
      struct armExtension {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::TalonFXInvertType::Clockwise;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto peakOutputForward = 1.0;
        constexpr static auto peakOutputReverse = -1.0;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::lifter::armExtension::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::lifter::armExtension::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::lifter::armExtension::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::lifter::armExtension::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::lifter::armExtension::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::lifter::armExtension::allowableError;
      };
      struct shoulderDrive {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::shoulderEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::lifter::shoulder::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::lifter::shoulder::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::lifter::shoulder::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::lifter::shoulder::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::lifter::shoulder::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::lifter::shoulder::allowableError;
      };
      struct shoulderFollower {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::FollowMaster;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode =
            argos_lib::status_frame_config::MotorPresetMode::FollowerFX;  // As follower, send less CAN traffic
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::shoulderEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::lifter::shoulder::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::lifter::shoulder::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::lifter::shoulder::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::lifter::shoulder::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::lifter::shoulder::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::lifter::shoulder::allowableError;
      };

      // Currently just generic drive with remote sensor
      struct wrist {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = true;
        constexpr static double peakOutputForward = 0.7;
        constexpr static double peakOutputReverse = -0.7;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::wristEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::lifter::wrist::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::lifter::wrist::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::lifter::wrist::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::lifter::wrist::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::lifter::wrist::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::lifter::wrist::allowableError;
      };
    }  // namespace lifter
    namespace intake {
      struct intake {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
        constexpr static auto continuousCurrentLimit = 20_A;
      };
    }  // namespace intake
    namespace bash_guard {
      struct extension {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::bash_guard::extension::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::bash_guard::extension::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::bash_guard::extension::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::bash_guard::extension::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::bash_guard::extension::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::bash_guard::extension::allowableError;
      };
    }  // namespace bash_guard
  }    // namespace comp_bot

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configurations specific to practice robot
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace practice_bot {
    namespace drive {
      using genericDrive = motorConfig::comp_bot::drive::genericDrive;

      struct frontLeftTurn {
        constexpr static auto inverted = motorConfig::comp_bot::drive::frontLeftTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::comp_bot::drive::frontLeftTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::comp_bot::drive::frontLeftTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::comp_bot::drive::frontLeftTurn::neutralMode;
        constexpr static auto voltCompSat = motorConfig::comp_bot::drive::frontLeftTurn::voltCompSat;
        constexpr static auto statusFrameMotorMode = motorConfig::comp_bot::drive::frontLeftTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::frontLeftEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::comp_bot::drive::frontLeftTurn::remoteFilter0_type;
        constexpr static auto pid0_selectedSensor = motorConfig::comp_bot::drive::frontLeftTurn::pid0_selectedSensor;
        constexpr static auto pid0_kP = motorConfig::comp_bot::drive::frontLeftTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::comp_bot::drive::frontLeftTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::comp_bot::drive::frontLeftTurn::pid0_kD;
        constexpr static auto pid0_kF = motorConfig::comp_bot::drive::frontLeftTurn::pid0_kF;
        constexpr static auto pid0_iZone = motorConfig::comp_bot::drive::frontLeftTurn::pid0_iZone;
        constexpr static auto pid0_allowableError = motorConfig::comp_bot::drive::frontLeftTurn::pid0_allowableError;
      };
      struct frontRightTurn {
        constexpr static auto inverted = motorConfig::comp_bot::drive::frontRightTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::comp_bot::drive::frontRightTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::comp_bot::drive::frontRightTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::comp_bot::drive::frontRightTurn::neutralMode;
        constexpr static auto voltCompSat = motorConfig::comp_bot::drive::frontRightTurn::voltCompSat;
        constexpr static auto statusFrameMotorMode = motorConfig::comp_bot::drive::frontRightTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::frontRightEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::comp_bot::drive::frontRightTurn::remoteFilter0_type;
        constexpr static auto pid0_selectedSensor = motorConfig::comp_bot::drive::frontRightTurn::pid0_selectedSensor;
        constexpr static auto pid0_kP = motorConfig::comp_bot::drive::frontRightTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::comp_bot::drive::frontRightTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::comp_bot::drive::frontRightTurn::pid0_kD;
        constexpr static auto pid0_kF = motorConfig::comp_bot::drive::frontRightTurn::pid0_kF;
        constexpr static auto pid0_iZone = motorConfig::comp_bot::drive::frontRightTurn::pid0_iZone;
        constexpr static auto pid0_allowableError = motorConfig::comp_bot::drive::frontRightTurn::pid0_allowableError;
      };
      struct backRightTurn {
        constexpr static auto inverted = motorConfig::comp_bot::drive::backRightTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::comp_bot::drive::backRightTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::comp_bot::drive::backRightTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::comp_bot::drive::backRightTurn::neutralMode;
        constexpr static auto voltCompSat = motorConfig::comp_bot::drive::backRightTurn::voltCompSat;
        constexpr static auto statusFrameMotorMode = motorConfig::comp_bot::drive::backRightTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::backRightEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::comp_bot::drive::backRightTurn::remoteFilter0_type;
        constexpr static auto pid0_selectedSensor = motorConfig::comp_bot::drive::backRightTurn::pid0_selectedSensor;
        constexpr static auto pid0_kP = motorConfig::comp_bot::drive::backRightTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::comp_bot::drive::backRightTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::comp_bot::drive::backRightTurn::pid0_kD;
        constexpr static auto pid0_kF = motorConfig::comp_bot::drive::backRightTurn::pid0_kF;
        constexpr static auto pid0_iZone = motorConfig::comp_bot::drive::backRightTurn::pid0_iZone;
        constexpr static auto pid0_allowableError = motorConfig::comp_bot::drive::backRightTurn::pid0_allowableError;
      };
      struct backLeftTurn {
        constexpr static auto inverted = motorConfig::comp_bot::drive::backLeftTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::comp_bot::drive::backLeftTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::comp_bot::drive::backLeftTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::comp_bot::drive::backLeftTurn::neutralMode;
        constexpr static auto voltCompSat = motorConfig::comp_bot::drive::backLeftTurn::voltCompSat;
        constexpr static auto statusFrameMotorMode = motorConfig::comp_bot::drive::backLeftTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::backLeftEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::comp_bot::drive::backLeftTurn::remoteFilter0_type;
        constexpr static auto pid0_selectedSensor = motorConfig::comp_bot::drive::backLeftTurn::pid0_selectedSensor;
        constexpr static auto pid0_kP = motorConfig::comp_bot::drive::backLeftTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::comp_bot::drive::backLeftTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::comp_bot::drive::backLeftTurn::pid0_kD;
        constexpr static auto pid0_kF = motorConfig::comp_bot::drive::backLeftTurn::pid0_kF;
        constexpr static auto pid0_iZone = motorConfig::comp_bot::drive::backLeftTurn::pid0_iZone;
        constexpr static auto pid0_allowableError = motorConfig::comp_bot::drive::backLeftTurn::pid0_allowableError;
      };
    }  // namespace drive
    namespace lifter {
      using armExtension = motorConfig::comp_bot::lifter::armExtension;
      using shoulderDrive = motorConfig::comp_bot::lifter::shoulderDrive;
      using wrist = motorConfig::comp_bot::lifter::wrist;
    }  // namespace lifter
    namespace intake {
      using intake = motorConfig::comp_bot::intake::intake;
    }  // namespace intake
    namespace bash_guard {
      using extension = motorConfig::comp_bot::bash_guard::extension;
    }  // namespace bash_guard
  }    // namespace practice_bot
}  // namespace motorConfig
