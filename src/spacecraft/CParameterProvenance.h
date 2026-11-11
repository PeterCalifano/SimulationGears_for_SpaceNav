/// @file CParameterProvenance.h
/// @brief Traceable provenance of immutable parameter blocks.
#pragma once

#include <string>

/// @brief Native spacecraft simulation models and concrete wrapper interfaces.
namespace simulation_gears
{
    /// @brief Source classification, distinct from validation readiness.
    enum class EParameterProvenance
    {
        PUBLISHED, ///< Transcribed from an identified source/version.
        DERIVED,   ///< Calculated deterministically from identified inputs.
        DESIGNED,  ///< Explicit synthetic choice or modelling assumption.
        PENDING    ///< Required information remains unestablished.
    };

    /// @brief Immutable provenance carried at configuration boundaries, outside hot loops.
    class CParameterProvenance
    {
      public:
        /// @brief Record the class and a source, calculation, assumption, or missing-data reason.
        /// @throws std::invalid_argument For an empty reference or invalid enum.
        /// @param kind Published, derived, designed or pending provenance classification.
        /// @param reference Nonempty source/version, derivation, assumption or missing-data explanation.
        CParameterProvenance(EParameterProvenance kind, const std::string &reference);

        /// @brief Return the classification.
        /// @return The classification.
        EParameterProvenance getKind() const
        {
            return kind_;
        }

        /// @brief Return the traceable source or declared assumption.
        /// @return The traceable source or declared assumption.
        std::string getReference() const
        {
            return reference_;
        }

      private:
        EParameterProvenance kind_;
        std::string reference_;
    };
} // namespace simulation_gears
