/// @file CLogger.h
/// @brief Declares the dependency-free SimulationGears component logger.

#pragma once

#include <atomic>
#include <concepts>
#include <cstdint>
#include <iostream>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace simulation_gears::logging
{
    /** @brief Ordered logging threshold used by CLogger. */
    enum class ELogLevel : std::uint8_t
    {
        Quiet = 0,
        Critical = 1,
        Error = 2,
        Warning = 3,
        Info = 4,
        Debug = 5,
        Trace = 6
    };

    /** @brief Select whether CLogger emits ANSI terminal color sequences. */
    enum class ELogColorMode : std::uint8_t
    {
        Disabled = 0,
        Enabled = 1
    };

    /** @brief A value that can be appended to a standard output stream. */
    template <typename TValue>
    concept StreamInsertable = requires(std::ostream &objStream_, TValue &&value_) {
        objStream_ << std::forward<TValue>(value_);
    };

    /**
     * @brief Small dependency-free, component-scoped logger.
     *
     * Complete lines are serialized across all logger instances. Critical,
     * error, and warning messages use the diagnostic stream; info, debug, and
     * trace messages use the ordinary output stream. Callers that provide custom
     * streams must keep them alive for the logger's lifetime.
     */
    class CLogger final
    {
      public:
        /**
         * @brief Construct a logger for one component.
         * @param charComponentName Component printed in each line. An empty name
         * is replaced with `simulation_gears`.
         * @param enumLevel Initial verbosity threshold.
         * @param enumColorMode Explicit ANSI color policy.
         * @param objOutputStream Stream for info, debug, and trace messages.
         * @param objDiagnosticStream Stream for critical, error, and warning messages.
         */
        explicit CLogger(std::string charComponentName,
                         ELogLevel enumLevel = ELogLevel::Error,
                         ELogColorMode enumColorMode = ELogColorMode::Disabled,
                         std::ostream &objOutputStream = std::cout,
                         std::ostream &objDiagnosticStream = std::clog);

        CLogger(const CLogger &) = delete;
        CLogger &operator=(const CLogger &) = delete;
        CLogger(CLogger &&) = delete;
        CLogger &operator=(CLogger &&) = delete;
        ~CLogger() = default;

        /** @brief Change the active verbosity threshold. */
        void setLevel(ELogLevel enumLevel) noexcept;

        /** @brief Return the active verbosity threshold. */
        [[nodiscard]] ELogLevel getLevel() const noexcept;

        /** @brief Return true when a severity is enabled by the active threshold. */
        [[nodiscard]] bool shouldLog(ELogLevel enumSeverity) const noexcept;

        /**
         * @brief Parse a case-insensitive level name or numeric value from 0 to 6.
         * @param charLevelText Level text with optional surrounding ASCII whitespace.
         * @return Parsed level, or std::nullopt when the text is invalid.
         */
        [[nodiscard]] static std::optional<ELogLevel> tryParseLevel(std::string_view charLevelText);

        /**
         * @brief Apply a valid level from an environment variable.
         * @param charVariableName Environment variable to read.
         * @return True only when a valid value was found and applied.
         */
        bool setLevelFromEnvironment(
            std::string_view charVariableName = "SIMULATION_GEARS_LOG_LEVEL");

        /** @brief Emit a critical message when enabled. */
        template <StreamInsertable... TArgs>
        void critical(TArgs &&...args)
        {
            write_(ELogLevel::Critical, std::forward<TArgs>(args)...);
        }

        /** @brief Emit an error message when enabled. */
        template <StreamInsertable... TArgs>
        void error(TArgs &&...args)
        {
            write_(ELogLevel::Error, std::forward<TArgs>(args)...);
        }

        /** @brief Emit a warning message when enabled. */
        template <StreamInsertable... TArgs>
        void warning(TArgs &&...args)
        {
            write_(ELogLevel::Warning, std::forward<TArgs>(args)...);
        }

        /** @brief Emit an informational message when enabled. */
        template <StreamInsertable... TArgs>
        void info(TArgs &&...args)
        {
            write_(ELogLevel::Info, std::forward<TArgs>(args)...);
        }

        /** @brief Emit a debug message when enabled. */
        template <StreamInsertable... TArgs>
        void debug(TArgs &&...args)
        {
            write_(ELogLevel::Debug, std::forward<TArgs>(args)...);
        }

        /** @brief Emit a trace message when enabled. */
        template <StreamInsertable... TArgs>
        void trace(TArgs &&...args)
        {
            write_(ELogLevel::Trace, std::forward<TArgs>(args)...);
        }

      private:
        template <StreamInsertable... TArgs>
        void write_(ELogLevel enumSeverity, TArgs &&...args)
        {
            if (!shouldLog(enumSeverity))
            {
                return;
            }

            std::ostringstream objMessageStream_;
            (objMessageStream_ << ... << std::forward<TArgs>(args));
            writeMessage_(enumSeverity, objMessageStream_.str());
        }

        void writeMessage_(ELogLevel enumSeverity, std::string_view charMessage);
        [[nodiscard]] std::ostream &selectStream_(ELogLevel enumSeverity) const noexcept;
        [[nodiscard]] static std::string_view getLevelLabel_(ELogLevel enumSeverity) noexcept;
        [[nodiscard]] static std::string_view getColorCode_(ELogLevel enumSeverity) noexcept;

        std::string charComponentName_;
        std::atomic<ELogLevel> enumLevel_;
        ELogColorMode enumColorMode_;
        std::ostream &objOutputStream_;
        std::ostream &objDiagnosticStream_;
    };
} // namespace simulation_gears::logging
