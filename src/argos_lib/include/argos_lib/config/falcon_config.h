/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/current.h>
#include <units/time.h>

#include <iostream>

#include "argos_lib/config/config_types.h"
#include "compile_time_member_check.h"
#include "ctre/Phoenix.h"
#include "status_frame_config.h"

namespace argos_lib {
  namespace falcon_config {

    HAS_MEMBER(forwardLimit_deviceID)
    HAS_MEMBER(forwardLimit_normalState)
    HAS_MEMBER(forwardLimit_source)
    HAS_MEMBER(inverted)
    HAS_MEMBER(neutralDeadband)
    HAS_MEMBER(neutralMode)
    HAS_MEMBER(nominalOutputForward)
    HAS_MEMBER(nominalOutputReverse)
    HAS_MEMBER(peakOutputForward)
    HAS_MEMBER(peakOutputReverse)
    HAS_MEMBER(pid0_allowableError)
    HAS_MEMBER(pid0_iZone)
    HAS_MEMBER(pid0_kD)
    HAS_MEMBER(pid0_kF)
    HAS_MEMBER(pid0_kI)
    HAS_MEMBER(pid0_kP)
    HAS_MEMBER(pid0_selectedSensor)
    HAS_MEMBER(pid1_allowableError)
    HAS_MEMBER(pid1_iZone)
    HAS_MEMBER(pid1_kD)
    HAS_MEMBER(pid1_kF)
    HAS_MEMBER(pid1_kI)
    HAS_MEMBER(pid1_kP)
    HAS_MEMBER(remoteFilter0_addr)
    HAS_MEMBER(remoteFilter0_type)
    HAS_MEMBER(reverseLimit_deviceID)
    HAS_MEMBER(reverseLimit_normalState)
    HAS_MEMBER(reverseLimit_source)
    HAS_MEMBER(sensorPhase)
    HAS_MEMBER(supplyCurrentLimit)
    HAS_MEMBER(supplyCurrentThreshold)
    HAS_MEMBER(supplyCurrentThresholdTime)
    HAS_MEMBER(voltCompSat)
    HAS_MEMBER(statusFrameMotorMode)

    /**
     * @brief Configures a CTRE Falcon with only the fields provided.  All other fields
     *        are given the factory default values.
     *
     * @tparam T Structure containing any combination of the following members:
     *           - forwardLimit_deviceID
     *           - forwardLimit_normalState
     *           - forwardLimit_source
     *           - inverted
     *           - neutralDeadband
     *           - neutralMode
     *           - nominalOutputForward
     *           - nominalOutputReverse
     *           - peakOutputForward
     *           - peakOutputReverse
     *           - pid0_allowableError
     *           - pid0_iZone
     *           - pid0_kD
     *           - pid0_kF
     *           - pid0_kI
     *           - pid0_kP
     *           - pid0_selectedSensor
     *           - pid1_allowableError
     *           - pid1_iZone
     *           - pid1_kD
     *           - pid1_kF
     *           - pid1_kI
     *           - pid1_kP
     *           - remoteFilter0_addr
     *           - remoteFilter0_type
     *           - reverseLimit_deviceID
     *           - reverseLimit_normalState
     *           - reverseLimit_source
     *           - sensorPhase
     *           - supplyCurrentLimit
     *           - supplyCurrentThreshold
     *           - supplyCurrentThresholdTime
     *           - voltCompSat
     *           - statusFrameMotorMode
     * @param motorController Falcon object to configure
     * @param configTimeout Time to wait for response from Falcon
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename T>
    bool FalconConfig(TalonFX& motorController, units::millisecond_t configTimeout) {
      TalonFXConfiguration config;
      auto timeout = configTimeout.to<int>();

      if constexpr (has_inverted<T>{}) {
        motorController.SetInverted(T::inverted);
      }
      if constexpr (has_sensorPhase<T>{}) {
        motorController.SetSensorPhase(T::sensorPhase);
      }
      if constexpr (has_neutralMode<T>{}) {
        motorController.SetNeutralMode(T::neutralMode);
      }
      if constexpr (has_voltCompSat<T>{}) {
        constexpr units::volt_t voltage = T::voltCompSat;
        config.voltageCompSaturation = voltage.to<double>();
        motorController.EnableVoltageCompensation(true);
      } else {
        motorController.EnableVoltageCompensation(false);
      }
      if constexpr (has_remoteFilter0_addr<T>{} && has_remoteFilter0_type<T>{}) {
        ctre::phoenix::motorcontrol::can::FilterConfiguration filterConfig;
        filterConfig.remoteSensorDeviceID = T::remoteFilter0_addr.address;
        filterConfig.remoteSensorSource = T::remoteFilter0_type;
        config.remoteFilter0 = filterConfig;
      }
      if constexpr (has_nominalOutputForward<T>{}) {
        config.nominalOutputForward = T::nominalOutputForward;
      }
      if constexpr (has_nominalOutputReverse<T>{}) {
        config.nominalOutputReverse = T::nominalOutputReverse;
      }
      if constexpr (has_peakOutputForward<T>{}) {
        config.peakOutputForward = T::peakOutputForward;
      }
      if constexpr (has_peakOutputReverse<T>{}) {
        config.peakOutputReverse = T::peakOutputReverse;
      }
      if constexpr (has_pid0_selectedSensor<T>{}) {
        config.primaryPID.selectedFeedbackSensor = T::pid0_selectedSensor;
      }
      if constexpr (has_pid0_kP<T>{}) {
        config.slot0.kP = T::pid0_kP;
      }
      if constexpr (has_pid0_kI<T>{}) {
        config.slot0.kI = T::pid0_kI;
      }
      if constexpr (has_pid0_kD<T>{}) {
        config.slot0.kD = T::pid0_kD;
      }
      if constexpr (has_pid0_kF<T>{}) {
        config.slot0.kF = T::pid0_kF;
      }
      if constexpr (has_pid0_iZone<T>{}) {
        config.slot0.integralZone = T::pid0_iZone;
      }
      if constexpr (has_pid0_allowableError<T>{}) {
        config.slot0.allowableClosedloopError = T::pid0_allowableError;
      }
      if constexpr (has_pid1_kP<T>{}) {
        config.slot1.kP = T::pid1_kP;
      }
      if constexpr (has_pid1_kI<T>{}) {
        config.slot1.kI = T::pid1_kI;
      }
      if constexpr (has_pid1_kD<T>{}) {
        config.slot1.kD = T::pid1_kD;
      }
      if constexpr (has_pid1_kF<T>{}) {
        config.slot1.kF = T::pid1_kF;
      }
      if constexpr (has_pid1_iZone<T>{}) {
        config.slot1.integralZone = T::pid1_iZone;
      }
      if constexpr (has_pid1_allowableError<T>{}) {
        config.slot1.allowableClosedloopError = T::pid1_allowableError;
      }
      if constexpr (has_supplyCurrentLimit<T>{} || has_supplyCurrentThreshold<T>{} ||
                    has_supplyCurrentThresholdTime<T>{}) {
        config.supplyCurrLimit.enable = true;
        if constexpr (has_supplyCurrentLimit<T>{}) {
          constexpr units::ampere_t currentLimit = T::supplyCurrentLimit;
          static_assert(currentLimit.to<double>() > 0, "Current limit must be positive");
          config.supplyCurrLimit.currentLimit = currentLimit.to<double>();
        }
        if constexpr (has_supplyCurrentThreshold<T>{}) {
          constexpr units::ampere_t currentThreshold = T::supplyCurrentThreshold;
          static_assert(currentThreshold.to<double>() > 0, "Current threshold must be positive");
          config.supplyCurrLimit.triggerThresholdCurrent = currentThreshold.to<double>();
        }
        if constexpr (has_supplyCurrentThresholdTime<T>{}) {
          constexpr units::second_t currentThresholdTime = T::supplyCurrentThresholdTime;
          static_assert(currentThresholdTime.to<double>() >= 0, "Current threshold time must be non-negative");
          config.supplyCurrLimit.triggerThresholdTime = currentThresholdTime.to<double>();
        }
      }
      if constexpr (has_forwardLimit_source<T>{} || has_forwardLimit_deviceID<T>{} ||
                    has_forwardLimit_normalState<T>{}) {
        if constexpr (has_forwardLimit_source<T>{}) {
          constexpr ctre::phoenix::motorcontrol::LimitSwitchSource source = T::forwardLimit_source;
          if constexpr (source != ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated &&
                        source != ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector) {
            static_assert(has_forwardLimit_deviceID<T>{}, "Forward limit switch requires remote source device ID");
          }
          if constexpr (source != ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated) {
            static_assert(has_forwardLimit_normalState<T>{} &&
                              T::forwardLimit_normalState != ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled,
                          "Forward limit switch configuration requires both source and normal state");
          }
          config.forwardLimitSwitchSource = T::forwardLimit_source;
        }
        if constexpr (has_forwardLimit_deviceID<T>{}) {
          static_assert(has_forwardLimit_source<T>{} &&
                            T::forwardLimit_source != ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated &&
                            T::forwardLimit_source != ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector,
                        "Forward limit switch device ID has no effect when limit source is not remote");
          config.forwardLimitSwitchDeviceID = T::forwardLimit_deviceID;
        }
        if constexpr (has_forwardLimit_normalState<T>{}) {
          if constexpr (T::forwardLimit_normalState != ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled) {
            static_assert(has_forwardLimit_source<T>{}, "Forward limit switch source required");
          }
          config.forwardLimitSwitchNormal = T::forwardLimit_normalState;
        }
      }
      if constexpr (has_reverseLimit_source<T>{} || has_reverseLimit_deviceID<T>{} ||
                    has_reverseLimit_normalState<T>{}) {
        if constexpr (has_reverseLimit_source<T>{}) {
          constexpr ctre::phoenix::motorcontrol::LimitSwitchSource source = T::reverseLimit_source;
          if constexpr (source != ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated &&
                        source != ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector) {
            static_assert(has_reverseLimit_deviceID<T>{}, "Reverse limit switch requires remote source device ID");
          }
          if constexpr (source != ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated) {
            static_assert(has_reverseLimit_normalState<T>{} &&
                              T::reverseLimit_normalState != ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled,
                          "Reverse limit switch configuration requires both source and normal state");
          }
          config.reverseLimitSwitchSource = T::reverseLimit_source;
        }
        if constexpr (has_reverseLimit_deviceID<T>{}) {
          static_assert(has_reverseLimit_source<T>{} &&
                            T::reverseLimit_source != ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated &&
                            T::reverseLimit_source != ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector,
                        "Reverse limit switch device ID has no effect when limit source is not remote");
          config.reverseLimitSwitchDeviceID = T::reverseLimit_deviceID;
        }
        if constexpr (has_reverseLimit_normalState<T>{}) {
          if constexpr (T::reverseLimit_normalState != ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled) {
            static_assert(has_reverseLimit_source<T>{}, "Reverse limit switch source required");
          }
          config.reverseLimitSwitchNormal = T::reverseLimit_normalState;
        }
      }
      if constexpr (has_neutralDeadband<T>{}) {
        static_assert(T::neutralDeadband >= 0.001, "Neutral deadband must be greater than 0.001 (0.1%)");
        static_assert(T::neutralDeadband <= 0.25, "Neutral deadband must be less than 0.25 (25%)");
        config.neutralDeadband = T::neutralDeadband;
      }

      if constexpr (has_statusFrameMotorMode<T>()) {
        argos_lib::status_frame_config::SetMotorStatusFrameRates(motorController, T::statusFrameMotorMode);
      }

      auto retVal = motorController.ConfigAllSettings(config, timeout);
      if (0 != retVal) {
        std::cout << "Error code (" << motorController.GetDeviceID() << "): " << retVal << '\n';
      }

      return 0 != retVal;
    }

    /**
     * @brief Configures a CTRE Falcon with configuration values according to specified robot instance.
     *
     * @tparam CompetitionConfig Configurations to use in competition robot instance
     * @tparam PracticeConfig Configurations to use in practice robot instance
     * @param motorController Falcon object to configure
     * @param configTimeout Time to wait for response from Falcon
     * @param instance Robot instance to use
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename CompetitionConfig, typename PracticeConfig>
    bool FalconConfig(WPI_TalonFX& motorController,
                      units::millisecond_t configTimeout,
                      argos_lib::RobotInstance instance) {
      switch (instance) {
        case argos_lib::RobotInstance::Competition:
          return FalconConfig<CompetitionConfig>(motorController, configTimeout);
          break;
        case argos_lib::RobotInstance::Practice:
          return FalconConfig<PracticeConfig>(motorController, configTimeout);
          break;
      }
      return false;
    }

  }  // namespace falcon_config
}  // namespace argos_lib
