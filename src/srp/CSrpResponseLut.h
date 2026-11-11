/// @file CSrpResponseLut.h
/// @brief Immutable SI response samples and the portable SRP artifact boundary.
#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <vector>

namespace simulation_gears
{
    /// @brief Supported force channels, independent of pressure, mass and attitude.
    enum class ESrpChannels { Scalar = 1, ScalarTransverse = 4 };

    /// @brief Construction provenance; not part of the numerical runtime state.
    struct SSrpLutProvenance
    {
        std::string geometrySha256;
        std::string descriptorSha256;
        std::string generator = "array input";
        bool selfShadowing = false;
        unsigned int shadowSamples = 0;
        double rayOffsetMetres = 0.0;
    };

    /// @brief Owned, validated full-sphere uniform samples with elevation within azimuth.
    /// @details Channels are contiguous at each node: Cr [-], optionally Tx, Ty, Tz [m^2].
    /// Azimuth spans [-180,180] deg and elevation [-90,90] deg, including both endpoints.
    /// No mass, pressure, torque or trajectory data are stored. Copies are disabled; share const
    /// ownership or move at preparation boundaries. Seam and pole samples must agree exactly.
    class CSrpResponseLut
    {
      public:
        /// @brief Validate and take ownership of active samples (no fixed-capacity padding).
        /// @param azimuthDegrees Uniform full-sphere azimuth axis, at least three nodes.
        /// @param elevationDegrees Uniform full-sphere elevation axis, at least three nodes.
        /// @param channels Scalar or scalar plus transverse force channels.
        /// @param samples Node-major values, indexed ((azimuth * elevationCount) + elevation) * channels.
        /// @param referenceAreaSquareMetres Positive reference area used to define Cr.
        /// @param provenance Traceable source and construction policy.
        /// @throws std::invalid_argument For malformed axes, values, units or transverse samples.
        CSrpResponseLut(std::vector<double> azimuthDegrees, std::vector<double> elevationDegrees,
                        ESrpChannels channels, std::vector<double> samples,
                        double referenceAreaSquareMetres, SSrpLutProvenance provenance = {});

        // Disable copies; moves transfer the owned storage.
        CSrpResponseLut(const CSrpResponseLut&) = delete;
        CSrpResponseLut& operator=(const CSrpResponseLut&) = delete;
        CSrpResponseLut(CSrpResponseLut&&) noexcept = default;
        CSrpResponseLut& operator=(CSrpResponseLut&&) noexcept = default;

        /// @brief Return the immutable azimuth axis [deg].
        const std::vector<double>& azimuthDegrees() const noexcept { return azimuthDegrees_; }
        /// @brief Return the immutable elevation axis [deg].
        const std::vector<double>& elevationDegrees() const noexcept { return elevationDegrees_; }
        /// @brief Return the immutable contiguous samples.
        const std::vector<double>& samples() const noexcept { return samples_; }
        /// @brief Return the force channel selection.
        ESrpChannels channels() const noexcept { return channels_; }
        /// @brief Return the reference area [m^2].
        double referenceAreaSquareMetres() const noexcept { return referenceAreaSquareMetres_; }
        /// @brief Return preparation provenance.
        const SSrpLutProvenance& provenance() const noexcept { return provenance_; }

      private:
        std::vector<double> azimuthDegrees_;
        std::vector<double> elevationDegrees_;
        ESrpChannels channels_;
        std::vector<double> samples_;
        double referenceAreaSquareMetres_;
        SSrpLutProvenance provenance_;
    };

    /// @brief Load a SimulationGears.SrpResponseLut.v1 JSON artifact into owned numerical storage.
    /// @param path Input JSON file. All schema, shape and physical invariants are checked.
    /// @return Validated immutable force table.
    /// @throws std::runtime_error For I/O or JSON/schema errors; invalid_argument for numeric errors.
    CSrpResponseLut LoadSrpResponseLut(const std::filesystem::path& path);

    /// @brief Write full-precision JSON with explicit units, layout, channel order and provenance.
    /// @param lut Validated table to serialize.
    /// @param path Output file; existing files are replaced only by an explicit call.
    /// @throws std::runtime_error For an output failure.
    void SaveSrpResponseLut(const CSrpResponseLut& lut, const std::filesystem::path& path);
} // namespace simulation_gears
