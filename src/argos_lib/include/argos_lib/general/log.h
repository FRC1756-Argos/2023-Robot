/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

/*
  Contains functions / classes useful for logging runtime information
*/

#pragma once

#include <fmt/format.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

#include <cstdarg>
#include <iostream>
#include <string>
#include <string_view>

#include "units/base.h"

namespace {
  /// @brief Represents a log level of either information, or error
  enum LogLevel { INFO, WARN, ERR };
}  // namespace

namespace argos_lib {

  /// @brief Log to the console in a clean, repeatable manner
  class ArgosLogger {
   public:
    ArgosLogger() = delete;
    explicit ArgosLogger(std::string tag)
        : m_tag{tag}
        , m_log{frc::DataLogManager::GetLog()}
        , m_infoLog{m_log, GetLogKey(m_tag, LogLevel::INFO)}
        , m_warnLog{m_log, GetLogKey(m_tag, LogLevel::WARN)}
        , m_errLog{m_log, GetLogKey(m_tag, LogLevel::ERR)} {}
    /// @brief Log Info
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    void LogI(const char* fmt, ...) const {
      va_list lst;
      va_start(lst, fmt);
      Log(LogLevel::INFO, fmt, lst);
    }

    /// @brief Log information with specified behavior
    /// @param ntEn True if logging to Network Tables
    /// @param fsEn True if logging to robot fs
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    void LogI(const bool& ntEn, const bool& fsEn, const char* fmt, ...) {
      va_list lst;
      va_start(lst, fmt);
      Log(LogLevel::INFO, ntEn, fsEn, fmt, lst);
    }

    /// @brief Log Warning
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    void LogW(const char* fmt, ...) const {
      va_list lst;
      va_start(lst, fmt);
      Log(LogLevel::WARN, fmt, lst);
    }

    /// @brief Log warning with specified behavior
    /// @param ntEn True if logging to Network Tables
    /// @param fsEn True if logging to robot fs
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    void LogW(const bool& ntEn, const bool& fsEn, const char* fmt, ...) {
      va_list lst;
      va_start(lst, fmt);
      Log(LogLevel::WARN, ntEn, fsEn, fmt, lst);
    }

    /// @brief Log Error
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    void LogE(const char* fmt, ...) const {
      va_list lst;
      va_start(lst, fmt);
      Log(LogLevel::ERR, fmt, lst);
    }

    /// @brief Log error with specified behavior
    /// @param ntEn True if logging to Network Tables
    /// @param fsEn True if logging to robot fs
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    void LogE(const bool& ntEn, const bool& fsEn, const char* fmt, ...) {
      va_list lst;
      va_start(lst, fmt);
      Log(LogLevel::ERR, ntEn, fsEn, fmt, lst);
    }

    /// @brief Configures logging to Network Tables
    /// @param ntEn True to enable NT logging, False to disable
    void SetNTEnabled(const bool& ntEn) { m_ntEnabled = ntEn; }

    /// @brief Configures logging to robot file system
    /// @param ntEn True to enable fs logging, False to disable
    void SetFSEnabled(const bool& fsEn) { m_fsEnabled = fsEn; }

   private:
    static std::string_view GetLogKey(std::string_view tag, LogLevel lvl) {
      std::string_view sLvl = "NOLVL";

      switch (lvl) {
        case LogLevel::INFO:
          sLvl = "INFO";
          break;
        case LogLevel::WARN:
          sLvl = "WARN";
          break;
        case LogLevel::ERR:
          sLvl = "ERR";
          break;

        default:
          // lvl didn't match any log levles, so set to NO LEVEL
          sLvl = "NOLVL";
          break;
      }

      return fmt::format("robotErrors/{}/{}", tag, sLvl);
    }

    /// @brief Private function that logs
    /// @param level Log level which changes behavior of logging, and tag
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    void Log(LogLevel level, const char* fmt, va_list lst) const {
      std::string_view ntKey;
      std::string_view fsKey;
      switch (level) {
        case LogLevel::INFO:
          std::fprintf(stdout, "[%s]_INFO", m_tag.c_str());
          ntKey = fmt::format("{}/{}", m_tag, "INFOS");
          break;

        case LogLevel::WARN:
          std::fprintf(stdout, "[%s]_WARN", m_tag.c_str());
          break;

        case LogLevel::ERR:
          std::fprintf(stderr, "[%s_ERROR]", m_tag.c_str());
          break;
        default:
          std::fprintf(stdout, "[%s]_NOLVL", m_tag.c_str());
          break;
      }
      std::vfprintf(stdout, fmt, lst);
    }

    /// @brief Log with specified temporary configuration
    /// @param level Log level to log at, determines behavior
    /// @param ntEn True if logging to network tables is enabled
    /// @param fsEn True if logging to robot fs is enabled
    /// @param fmt Input log message
    /// @param ... fmt args for log message
    void Log(LogLevel level, const bool& ntEn, const bool& fsEn, const char* fmt, va_list lst) {
      // Save old config
      bool ntPrev = m_ntEnabled;
      bool fsPrev = m_fsEnabled;
      // Set config
      SetNTEnabled(ntEn);
      SetFSEnabled(fsEn);

      // Log
      Log(level, fmt, lst);

      // Set back old config
      SetNTEnabled(ntPrev);
      SetFSEnabled(fsPrev);
    }

    std::string m_tag;
    bool m_ntEnabled;  ///< True if logging to Network Tables is enabled
    bool m_fsEnabled;  ///< True if logging to file system is enabled
    wpi::log::DataLog& m_log;
    wpi::log::StringArrayLogEntry m_infoLog;
    wpi::log::StringArrayLogEntry m_warnLog;
    wpi::log::StringArrayLogEntry m_errLog;
  };
}  // namespace argos_lib
