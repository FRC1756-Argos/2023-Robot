/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <networktables/NetworkTableInstance.h>

#include <memory>
#include <string>

namespace argos_lib {
  /**
   * @brief Subscribes to Network Tables entry updates and calls a specified callback to use the new value.
   */
  class NTSubscriber {
   public:
    /**
     * @brief Construct a new NTSubscriber object
     *
     * @param tableName Name of the table containing keys to watch
     */
    explicit NTSubscriber(const std::string& tableName);

    /**
     * @brief Register a new listener for the specified key and send values to callback on update
     *
     * @param keyName Key to listen for updates.  Will be at tableName/keyName (keyName can have more slashes)
     * @param onUpdateCallback Callback to run when keyName changes
     * @param defaultValue Default value to initialize key if it doesn't exist yet
     * @param forceUpdate When true, update network tables value to default even if another value is already set
     */
    void AddMonitor(const std::string& keyName,
                    std::function<void(double)> onUpdateCallback,
                    const double defaultValue = 0.0,
                    const bool forceUpdate = true);

   private:
    std::shared_ptr<nt::NetworkTable> m_pntTable;  ///< Table with monitored keys
  };

}  // namespace argos_lib
