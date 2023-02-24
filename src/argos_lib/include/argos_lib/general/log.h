/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <fmt/format.h>
#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringArrayTopic.h>
#include <wpi/DataLog.h>

#include <iostream>
#include <string>
#include <string_view>

namespace argos_lib {

  enum LogLevel { INFO, WARN, ERR };

  class ArgosLogger {
   public:
    explicit ArgosLogger(const std::string& tag)
        : m_tag{tag}, m_fsEnabled{false}, m_log{frc::DataLogManager::GetLog()} {}

    /// @brief Log Info
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    template <class... Args>
    void LogI(fmt::string_view fmt, Args&&... args) const {
      Log(LogLevel::INFO, fmt, fmt::make_format_args(args...));
    }

    /// @brief Log Warning
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    template <class... Args>
    void LogW(fmt::string_view fmt, Args&&... args) const {
      Log(LogLevel::WARN, fmt, fmt::make_format_args(args...));
    }

    /// @brief Log Error
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    template <class... Args>
    void LogE(fmt::string_view fmt, Args&&... args) const {
      Log(LogLevel::ERR, fmt, fmt::make_format_args(args...));
    }

    void SetFSEnabled(bool fsEn) { m_fsEnabled = fsEn; }

   private:
    static std::string GetLogKey(const std::string& tag, LogLevel lvl) {
      fmt::string_view strLevel;
      switch (lvl) {
        case LogLevel::INFO:
          strLevel = "INFO";
          break;
        case LogLevel::WARN:
          strLevel = "WARN";
          break;
        case LogLevel::ERR:
          strLevel = "ERR";
          break;

        default:
          strLevel = "NOLVL";
          break;
      }

      return fmt::format("robotErrors/{}/{}", tag, strLevel);
    }

    /// @brief Private function that logs
    /// @param level Log level which changes behavior of logging, and tag
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    void Log(LogLevel level, fmt::string_view format, fmt::format_args args) const {
      std::string tag;
      std::string message = fmt::vformat(format, args);
      switch (level) {
        case LogLevel::INFO:
          tag = fmt::format("{}_INFO ", m_tag);
          std::cout << std::string(tag + message).c_str();
          break;

        case LogLevel::WARN:
          tag = fmt::format("{}_WARN ", m_tag);
          std::cout << std::string(tag + message).c_str();
          break;

        case LogLevel::ERR:
          tag = fmt::format("{}_ERROR ", m_tag);
          std::cerr << std::string(tag + message).c_str();
          break;

        default:
          tag = fmt::format("{}_NOLVL ", m_tag);
          std::cout << std::string(tag + message).c_str();
          break;
      }

      // Saves to log on the RoboRio
      if (m_fsEnabled) {
        wpi::log::StringArrayLogEntry logEntry(m_log, GetLogKey(m_tag, level));
        logEntry.Append({std::string_view(tag + message)});
      }
    }

    std::string m_tag;
    bool m_fsEnabled = false;
    wpi::log::DataLog& m_log;
  };
}  // namespace argos_lib
