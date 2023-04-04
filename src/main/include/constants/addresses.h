/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/I2C.h>

#include "argos_lib/config/config_types.h"

namespace address {
  namespace comp_bot {
    struct drive {
      constexpr static argos_lib::CANAddress frontLeftDrive{1, "drive"};
      constexpr static argos_lib::CANAddress frontLeftTurn{2, "drive"};
      constexpr static argos_lib::CANAddress frontRightDrive{3, "drive"};
      constexpr static argos_lib::CANAddress frontRightTurn{4, "drive"};
      constexpr static argos_lib::CANAddress backRightDrive{5, "drive"};
      constexpr static argos_lib::CANAddress backRightTurn{6, "drive"};
      constexpr static argos_lib::CANAddress backLeftDrive{7, "drive"};
      constexpr static argos_lib::CANAddress backLeftTurn{8, "drive"};
    };
    struct lifter {
      constexpr static argos_lib::CANAddress frontShoulder{9, "rio"};
      constexpr static argos_lib::CANAddress backShoulder{10, "rio"};
      constexpr static argos_lib::CANAddress armExtension{11, "rio"};
      constexpr static argos_lib::CANAddress wrist{12, "rio"};
    };
    struct intake {
      constexpr static argos_lib::CANAddress intakeMotor{13, "rio"};
    };
    struct bash_guard {
      constexpr static argos_lib::CANAddress extension{14, "rio"};
    };
    struct encoders {
      constexpr static argos_lib::CANAddress frontLeftEncoder{1, "drive"};
      constexpr static argos_lib::CANAddress frontRightEncoder{2, "drive"};
      constexpr static argos_lib::CANAddress backRightEncoder{3, "drive"};
      constexpr static argos_lib::CANAddress backLeftEncoder{4, "drive"};

      // Lifter encoders
      constexpr static argos_lib::CANAddress shoulderEncoder{5, "rio"};
      constexpr static argos_lib::CANAddress armExtenderEncoder{6, "rio"};
      constexpr static argos_lib::CANAddress wristEncoder{7, "rio"};
    };
    struct controllers {
      constexpr static const char driver = 0;
      constexpr static const char secondary = 1;
      constexpr static const char buttonBox = 2;
    };
    struct solenoids {};
    struct sensors {
      constexpr static argos_lib::CANAddress pigeonIMU{1, "drive"};
      constexpr static const char tofCubeIntake = 1;
      constexpr static const char tofConeIntakeLeft = 2;
      constexpr static const char tofConeIntakeRight = 3;
    };
    struct led {
      constexpr static argos_lib::CANAddress CANdle{1, "drive"};
    };
  }  // namespace comp_bot
  namespace practice_bot {
    using drive = address::comp_bot::drive;
    using lifter = address::comp_bot::lifter;
    using intake = address::comp_bot::intake;
    using bash_guard = address::comp_bot::bash_guard;
    using encoders = address::comp_bot::encoders;
    using controllers = address::comp_bot::controllers;
    using solenoids = address::comp_bot::solenoids;
    using sensors = address::comp_bot::sensors;
    using led = address::comp_bot::led;
  }  // namespace practice_bot

}  // namespace address
