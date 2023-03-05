/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_over_charging_station.h"

#include <argos_lib/general/angle_utils.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/drive_until_pitch.h"
#include "commands/drive_until_pitch_rate.h"

DriveOverShargingStation::DriveOverShargingStation(SwerveDriveSubsystem* drive,
                                                   units::degree_t approachAngle,
                                                   units::degree_t robotYaw)
    : m_subCommandFailed{false}
    , m_pDrive{drive}
    , m_approachAngle{approachAngle}
    , m_robotYawAngle{robotYaw}
    , m_approachForward{units::math::abs(
                            argos_lib::angle::ConstrainAngle(robotYaw - approachAngle, -180_deg, 180_deg)) < 90_deg}
    , m_initialPitchSign{m_approachForward ? 1 : -1}
    , m_commands{
          DriveUntilPitch{drive,
                          approachAngle,
                          0.3,
                          0.0,
                          thresholds::robotHitChargingStationPitch * m_initialPitchSign,
                          m_approachForward ? ApproachDirection::Increasing : ApproachDirection::Decreasing,
                          2_s}
              .FinallyDo([this](bool cancelled) { m_subCommandFailed = m_subCommandFailed || cancelled; })
              .AndThen(
                  DriveUntilPitch{drive,
                                  approachAngle,
                                  0.5,
                                  0.3,
                                  thresholds::robotClimbPitch * m_initialPitchSign,
                                  m_approachForward ? ApproachDirection::Increasing : ApproachDirection::Decreasing,
                                  2_s}
                      .Unless([this]() { return m_subCommandFailed; })
                      .FinallyDo([this](bool cancelled) { m_subCommandFailed = m_subCommandFailed || cancelled; }))
              .AndThen(

                  DriveUntilPitchRate{drive,
                                      approachAngle,
                                      0.25,
                                      0.5,
                                      thresholds::robotTippingPitchRate * m_initialPitchSign,
                                      m_approachForward ? ApproachDirection::Decreasing : ApproachDirection::Increasing,
                                      2_s}
                      .Unless([this]() { return m_subCommandFailed; })
                      .FinallyDo([this](bool cancelled) { m_subCommandFailed = m_subCommandFailed || cancelled; }))
              .AndThen(
                  DriveUntilPitch{drive,
                                  approachAngle,
                                  0.1,
                                  0.25,
                                  thresholds::robotClimbPitch * m_initialPitchSign * -1,
                                  m_approachForward ? ApproachDirection::Decreasing : ApproachDirection::Increasing,
                                  2_s}
                      .Unless([this]() { return m_subCommandFailed; })
                      .FinallyDo([this](bool cancelled) { m_subCommandFailed = m_subCommandFailed || cancelled; }))
              .AndThen(

                  DriveUntilPitch{drive,
                                  approachAngle,
                                  0.3,
                                  0.1,
                                  thresholds::robotHitChargingStationPitch * m_initialPitchSign * -1,
                                  m_approachForward ? ApproachDirection::Increasing : ApproachDirection::Decreasing,
                                  2_s}
                      .Unless([this]() { return m_subCommandFailed; })
                      .FinallyDo([this](bool cancelled) { m_subCommandFailed = m_subCommandFailed || cancelled; }))} {}

// Called when the command is initially scheduled.
void DriveOverShargingStation::Initialize() {
  m_subCommandFailed = false;
  m_commands.Schedule();
}

// Called repeatedly when this Command is scheduled to run
void DriveOverShargingStation::Execute() {
  if (!m_commands.IsScheduled()) {
    Cancel();
    return;
  }
}

// Called once the command ends or is interrupted.
void DriveOverShargingStation::End(bool interrupted) {
  if (interrupted) {
    m_commands.Cancel();
  }
  m_pDrive->StopDrive();
}

// Returns true when the command should end.
bool DriveOverShargingStation::IsFinished() {
  return m_commands.get()->IsFinished();
}
