/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "argos_lib/general/swerve_utils.h"
#include "wpi/fs.h"

/**
 * @brief Accesses swerve module home positions in a file saved on the RoboRIO
 */
class FileSystemHomingStorage : public argos_lib::swerve::SwerveHomeStorageInterface {
 public:
  /**
   * @brief Construct a new File System Homing Storage object
   *
   * @param swerveHomesPath File path relative to home directory to save into and load from
   */
  explicit FileSystemHomingStorage(const fs::path& swerveHomesPath);
  /**
   * @brief Save positions as new homes
   *
   * @param homePosition Positions that represent 0 degree module orientations
   * @return true if save successful, false otherwise
   */
  bool Save(const argos_lib::swerve::SwerveModulePositions& homePosition) override;
  /**
   * @brief Load absolute positions that represent 0 degree module orientations
   *
   * @return Saved module positions if they exist, otherwise std::nullopt to indicate failure
   */
  std::optional<argos_lib::swerve::SwerveModulePositions> Load() override;

 private:
  /**
   * @brief Get the path of the file to load from and save to
   *
   * @return fs::path Absolute path of persistent storage file
   */
  fs::path GetFilePath();
  const fs::path m_swerveHomesPath;  ///< Path of persistent storage file relative to home directory
};
