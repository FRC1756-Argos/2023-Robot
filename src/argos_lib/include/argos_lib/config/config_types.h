/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <string_view>

namespace argos_lib {
  /**
   * @brief Differentiates between practice robot and competition robot
   */
  enum struct RobotInstance {
    Competition,  ///< Competition robot
    Practice      ///< Practice robot
  };

  /**
   * @brief Detect robot instance
   *
   * @return RobotInstance
   */
  RobotInstance GetRobotInstance();

  struct CANAddress {
    int address;
    std::string_view busName;

    CANAddress() = delete;
    constexpr CANAddress(int address, const std::string_view& busName = "rio") : address(address), busName(busName) {}
  };
}  // namespace argos_lib
