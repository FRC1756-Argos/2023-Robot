/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/config/status_frame_config.h"

void argos_lib::status_frame_config::SetMotorStatusFrameRates(
    BaseTalon& motor, argos_lib::status_frame_config::MotorPresetMode motorMode) {
  uint8_t generalStatusPeriod = 30;         // Status 1
  uint8_t feedback0Period = 30;             // Status 2
  uint8_t quadraturePeriod = 200;           // Status 3
  uint8_t ainTempVBatPeriod = 200;          // Status 4
  uint8_t miscPeriod = 200;                 // Status 6
  uint8_t commStatusPeriod = 50;            // Status 7
  uint8_t pulseWidthPeriod = 200;           // Status 8
  uint8_t motionProfBufferPeriod = 255;     // Status 9
  uint8_t motionProfTargetPeriod = 255;     // Status 10
  uint8_t gadgeteerPeriod = 255;            // Status 11
  uint8_t feedback1Period = 255;            // Status 12
  uint8_t primaryPIDFPeriod = 200;          // Status 13
  uint8_t auxPIDFPeriod = 200;              // Status 14
  uint8_t firmwareAPIStatusPeriod = 255;    // Status 15
  uint8_t auxMotionProfTargetPeriod = 255;  // Status 17
  uint8_t brushlessStatusPeriod = 255;      // Status ??

  switch (motorMode) {
    case argos_lib::status_frame_config::MotorPresetMode::BasicFX:
      brushlessStatusPeriod = 250;
      [[fallthrough]];
    case argos_lib::status_frame_config::MotorPresetMode::Basic:
      generalStatusPeriod = 125;
      feedback0Period = 125;
      break;
    case argos_lib::status_frame_config::MotorPresetMode::LeaderFX:
      brushlessStatusPeriod = 200;
      [[fallthrough]];
    case argos_lib::status_frame_config::MotorPresetMode::Leader:
      break;
    case argos_lib::status_frame_config::MotorPresetMode::FollowerFX:
      brushlessStatusPeriod = 250;
      [[fallthrough]];
    case argos_lib::status_frame_config::MotorPresetMode::Follower:
      generalStatusPeriod = 200;
      feedback0Period = 200;
      break;
    case argos_lib::status_frame_config::MotorPresetMode::MotionProfilingFX:
      brushlessStatusPeriod = 200;
      [[fallthrough]];
    case argos_lib::status_frame_config::MotorPresetMode::MotionProfiling:
      motionProfBufferPeriod = 40;
      motionProfTargetPeriod = 40;
      feedback0Period = 100;
      break;
    case argos_lib::status_frame_config::MotorPresetMode::TuningFX:
      brushlessStatusPeriod = 200;
      [[fallthrough]];
    case argos_lib::status_frame_config::MotorPresetMode::Tuning:
      feedback0Period = 100;
      break;
  }

  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_1_General, generalStatusPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_2_Feedback0, feedback0Period);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_3_Quadrature, quadraturePeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_4_AinTempVbat, ainTempVBatPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_6_Misc, miscPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_7_CommStatus, commStatusPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_8_PulseWidth, pulseWidthPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_9_MotProfBuffer, motionProfBufferPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_10_Targets, motionProfTargetPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_11_UartGadgeteer, gadgeteerPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_12_Feedback1, feedback1Period);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_13_Base_PIDF0, primaryPIDFPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_14_Turn_PIDF1, auxPIDFPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_15_FirmwareApiStatus, firmwareAPIStatusPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_17_Targets1, auxMotionProfTargetPeriod);
  motor.SetStatusFramePeriod(phoenix::motorcontrol::Status_Brushless_Current, brushlessStatusPeriod);
}
