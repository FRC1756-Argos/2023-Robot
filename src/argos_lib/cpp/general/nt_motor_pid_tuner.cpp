/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/nt_motor_pid_tuner.h"

#include <array>
#include <chrono>

using namespace std::literals::chrono_literals;

using argos_lib::ClosedLoopSensorConversions;
using argos_lib::NTMotorPIDTuner;

NTMotorPIDTuner::NTMotorPIDTuner(const std::string& tableName,
                                 std::initializer_list<ctre::phoenix::motorcontrol::can::BaseTalon*> motors,
                                 uint pidSlot,
                                 ClosedLoopSensorConversions sensorConversions)
    : m_updateSubscriber{tableName}
    , m_pMotors{motors}
    , m_pidSlot{pidSlot}
    , m_pntTable(nt::NetworkTableInstance::GetDefault().GetTable(tableName))
    , m_sensorConversions{sensorConversions}
    , m_threadMutex()
    , m_threadStopCv()
    , m_statusUpdateThread{[this]() { UpdateClosedLoopMonitoringThread(); }} {
  m_updateSubscriber.AddMonitor(
      "tunes/kP",
      [this](double newVal) {
        for (auto motor : m_pMotors) {
          motor->Config_kP(m_pidSlot, newVal, 50);
        }
      },
      m_pMotors.front()->ConfigGetParameter(eProfileParamSlot_P, m_pidSlot));
  m_updateSubscriber.AddMonitor(
      "tunes/kI",
      [this](double newVal) {
        for (auto motor : m_pMotors) {
          motor->Config_kI(m_pidSlot, newVal, 50);
        }
      },
      m_pMotors.front()->ConfigGetParameter(eProfileParamSlot_I, m_pidSlot));
  m_updateSubscriber.AddMonitor(
      "tunes/kD",
      [this](double newVal) {
        for (auto motor : m_pMotors) {
          motor->Config_kD(m_pidSlot, newVal, 50);
        }
      },
      m_pMotors.front()->ConfigGetParameter(eProfileParamSlot_D, m_pidSlot));
  m_updateSubscriber.AddMonitor(
      "tunes/kF",
      [this](double newVal) {
        for (auto motor : m_pMotors) {
          motor->Config_kF(m_pidSlot, newVal, 50);
        }
      },
      m_pMotors.front()->ConfigGetParameter(eProfileParamSlot_F, m_pidSlot));
  m_updateSubscriber.AddMonitor(
      "tunes/IZone",
      [this](double newVal) {
        for (auto motor : m_pMotors) {
          motor->Config_IntegralZone(m_pidSlot, newVal, 50);
        }
      },
      m_pMotors.front()->ConfigGetParameter(eProfileParamSlot_IZone, m_pidSlot));
  m_updateSubscriber.AddMonitor(
      "tunes/allowedError",
      [this](double newVal) {
        for (auto motor : m_pMotors) {
          motor->ConfigAllowableClosedloopError(m_pidSlot, newVal, 50);
        }
      },
      m_pMotors.front()->ConfigGetParameter(eProfileParamSlot_AllowableErr, m_pidSlot));
}

NTMotorPIDTuner::~NTMotorPIDTuner() {
  m_threadStopCv.notify_all();
  m_statusUpdateThread.join();
}

void NTMotorPIDTuner::UpdateClosedLoopMonitoringThread() {
  bool keepRunning = true;

  std::vector<double> setpoints;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> outputs;
  std::vector<double> errors;

  setpoints.reserve(m_pMotors.size());
  positions.reserve(m_pMotors.size());
  velocities.reserve(m_pMotors.size());
  outputs.reserve(m_pMotors.size());
  errors.reserve(m_pMotors.size());

  while (keepRunning) {
    std::unique_lock<std::mutex> lock{m_threadMutex};
    if (m_threadStopCv.wait_for(lock, 50ms) == std::cv_status::no_timeout) {
      keepRunning = false;
    } else {
      setpoints.clear();
      positions.clear();
      velocities.clear();
      outputs.clear();
      errors.clear();
      for (auto motor : m_pMotors) {
        const auto controlMode = motor->GetControlMode();
        if (controlMode == ctre::phoenix::motorcontrol::ControlMode::MotionMagic ||
            controlMode == ctre::phoenix::motorcontrol::ControlMode::MotionProfile ||
            controlMode == ctre::phoenix::motorcontrol::ControlMode::MotionProfileArc ||
            controlMode == ctre::phoenix::motorcontrol::ControlMode::Position ||
            controlMode == ctre::phoenix::motorcontrol::ControlMode::Velocity) {
          setpoints.push_back(motor->GetClosedLoopTarget(m_pidSlot) * m_sensorConversions.setpoint);
          errors.push_back(motor->GetClosedLoopError(m_pidSlot) * m_sensorConversions.setpoint);
        } else {
          setpoints.push_back(NAN);
          errors.push_back(NAN);
        }
        positions.push_back(motor->GetSelectedSensorPosition(m_pidSlot) * m_sensorConversions.position);
        velocities.push_back(motor->GetSelectedSensorVelocity(m_pidSlot) * m_sensorConversions.velocity);
        outputs.push_back(motor->GetMotorOutputPercent());
      }
      m_pntTable->PutNumberArray("status/setpoints", setpoints);
      m_pntTable->PutNumberArray("status/positions", positions);
      m_pntTable->PutNumberArray("status/velocities", velocities);
      m_pntTable->PutNumberArray("status/outputs", outputs);
      m_pntTable->PutNumberArray("status/errors", errors);
    }
  }
}
