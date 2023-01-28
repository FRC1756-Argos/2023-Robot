/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <fstream>
#include <iostream>

#include "units/base.h"
#include "wpi/fs.h"

/**
 * @brief Interface capable of saving and loading home positions from persistent storage
 *
 * @tparam T Type of the home position.  Should be a units type
 */
template <class T>
class HomingStorageInterface {
 public:
  /**
   * @brief Save home position to persistent storage
   *
   * @param homePosition Position to store
   * @return true Save successful
   * @return false Error saving
   */
  virtual bool Save(const T& homePosition) = 0;

  /**
   * @brief Load home position from persistent storage
   *
   * @return Poisition from persistent storage or std::nullopt if load failed or no positions were
   *         previously stored
   */
  [[nodiscard]] virtual std::optional<T> Load() = 0;
};

/**
 * @brief Saves and loads home positions from filesystem
 *
 * @tparam T Type of the home position.  Should be a units type
 */
template <class T>
class FSHomingStorage : public HomingStorageInterface<T> {
 public:
  /**
   * @brief Construct a new FSHomingStorage object
   *
   * @param homeFilePath Path to save homes to relative to internally-managed root directory
   */
  explicit FSHomingStorage(const fs::path& homeFilePath) : m_homesPath{homeFilePath} {};

  bool Save(const T& homePosition) override {
    try {
      bool success = true;
      std::ofstream configFile(GetFilePath(), std::ios::out);
      if (configFile.good()) {
        configFile << homePosition.template to<double>();
        if (!configFile.good()) {
          std::cout << "[ERROR] Could not write to config file\n";
          success = false;
        }
      } else {
        std::cout << "[ERROR] Could not open config file\n";
        success = false;
      }
      configFile.close();
      return success;
    } catch (...) {
      // Error accessing file
      std::cout << "[ERROR] Could not write to config file\n";
      return false;
    }
  }

  std::optional<T> Load() override {
    try {
      bool success = true;
      std::ifstream configFile(GetFilePath(), std::ios::in);
      double homePosition;
      configFile >> homePosition;

      configFile.close();
      if (success) {
        return units::make_unit<T>(homePosition);
      } else {
        return std::nullopt;
      }
    } catch (...) {
      // Error accessing file
      std::cout << "[ERROR] Could not read from config file\n";
      return std::nullopt;
    }
  }

 private:
  fs::path GetFilePath() {
    static const fs::path homeDir{"/home/lvuser"};
    static const fs::path configFile{homeDir / m_homesPath};

    std::cout << "############# Path: " << configFile << '\n';

    // Create empty file if it doesn't exist yet
    if (!fs::exists(configFile)) {
      fs::create_directories(configFile.parent_path());
      std::ofstream newFile(configFile);
      newFile.close();
    }

    return configFile;
  }

  const fs::path m_homesPath;
};
