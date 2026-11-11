/// @file CSrpPanelModel.h
/// @brief Prepared optical triangles and deterministic opaque self-shadowing in SI units.
#pragma once

#include "CSrpResponseLut.h"
#include <Eigen/Geometry>
#include <array>
#include <span>

namespace simulation_gears
{
    /// @brief One opaque source triangle in its declared record order, vertices in metres.
    using SrpTriangle = std::array<Eigen::Vector3d, 3>;

    /// @brief Optical side relative to the source triangle's winding.
    enum class ESrpOpticalSide { Front, Rear };

    /// @brief Visibility policy; shadow geometry remains two-sided in either optical direction.
    enum class ESrpShadowing { Disabled, OpaqueTriangles };

    /// @brief Explicit optical face; rear faces reuse geometry and never become extra occluders.
    struct SSrpOpticalFace
    {
        std::size_t sourceTriangle = 0;
        ESrpOpticalSide side = ESrpOpticalSide::Front;
        double diffuse = 0.0;
        double specular = 0.0;
    };

    /// @brief Shadow preparation options, outside force evaluation.
    struct SSrpShadowOptions
    {
        ESrpShadowing mode = ESrpShadowing::OpaqueTriangles;
        unsigned int subdivisionLevel = 3; ///< 4^level equal-area centroids: 64 by default.
        double rayOffsetMetres = 1e-8;
    };

    /// @brief Force/pressure [m^2] and its derivative with respect to the supplied direction vector.
    /// @details The derivative includes normalization, but holds visibility fractions fixed.
    struct SSrpPanelResponse
    {
        Eigen::Vector3d forcePerPressure = Eigen::Vector3d::Zero();
        Eigen::Matrix3d directionJacobian = Eigen::Matrix3d::Zero();
        Eigen::Vector3d torquePerPressure = Eigen::Vector3d::Zero(); ///< About mesh origin [m^3].
        bool regular = true; ///< False on an illumination terminator.
    };

    /// @brief Owned immutable preparation of optical faces, two-sided occluders and quadrature.
    /// @details Geometry uses source winding; no centroid-based normal reorientation occurs.
    class CSrpPanelModel
    {
      public:
        /// @brief Validate and prepare array input without any dependency on file parsers.
        /// @param triangles Opaque source triangles in metres, including non-optical occluders.
        /// @param opticalFaces Explicit source/side/optics mappings with no duplicates.
        /// @param shadow Shadow policy and quadrature level (0 through 5).
        /// @param provenance Source identities, retained by LUT construction.
        /// @throws std::invalid_argument For nonfinite/degenerate geometry or invalid optics/options.
        CSrpPanelModel(std::vector<SrpTriangle> triangles, std::vector<SSrpOpticalFace> opticalFaces,
                       SSrpShadowOptions shadow = {}, SSrpLutProvenance provenance = {});

        /// @brief Evaluate all illuminated optical faces using the established panel law.
        /// @param directionBody Finite, nonzero spacecraft-to-Sun vector; magnitude is ignored.
        /// @param frozenVisibility Optional fractions in optical-face order for partial validation.
        /// An empty span selects the configured shadow policy. Supplied fractions must lie in [0,1].
        /// @return Force, origin torque and frozen-visibility direction partials.
        /// @throws std::invalid_argument For invalid direction or fractions.
        SSrpPanelResponse evaluate(const Eigen::Vector3d& directionBody,
                                   std::span<const double> frozenVisibility = {}) const;

        /// @brief Return visibility fractions in optical-face order for a supplied Sun vector.
        /// @param directionBody Finite nonzero spacecraft-to-Sun vector.
        /// @return Owned fractions; back-facing optical surfaces receive zero.
        std::vector<double> visibility(const Eigen::Vector3d& directionBody) const;

        /// @brief Return the number of distinct opaque source triangles.
        std::size_t triangleCount() const noexcept { return triangles_.size(); }
        /// @brief Return the number of explicitly mapped optical faces.
        std::size_t opticalFaceCount() const noexcept { return opticalFaces_.size(); }
        /// @brief Return prepared source identities and quadrature settings.
        const SSrpLutProvenance& provenance() const noexcept { return provenance_; }

      private:
        /// @brief Optical face with its signed unit normal, centroid [m] and area [m^2].
        struct SPreparedOpticalFace
        {
            SSrpOpticalFace optics;
            Eigen::Vector3d normal;
            Eigen::Vector3d centre;
            double area;
        };

        /// @brief Return the fraction of a source triangle's samples that see the Sun direction.
        double visibleFraction(std::size_t sourceTriangle, const Eigen::Vector3d& direction) const;

        std::vector<SrpTriangle> triangles_;                ///< Opaque source triangles [m].
        std::vector<SPreparedOpticalFace> opticalFaces_;    ///< Optical faces in input order.
        std::vector<std::vector<Eigen::Vector3d>> samples_; ///< Shadow samples per source triangle [m].
        SSrpShadowOptions shadow_;
        SSrpLutProvenance provenance_;
    };

    /// @brief Construct scalar or transverse response samples with exact seam/pole copies.
    /// @param model Prepared spacecraft model; no trajectory or illumination history is fitted.
    /// @param referenceAreaSquareMetres Positive area defining the scalar response.
    /// @param angularStepDegrees Uniform spacing in [0.5,90] deg dividing 180 exactly.
    /// @param channels Requested force channels.
    /// @return Immutable table, with source and construction provenance.
    /// @throws std::invalid_argument For invalid spacing, area or channels.
    CSrpResponseLut BuildSrpResponseLut(const CSrpPanelModel& model, double referenceAreaSquareMetres,
                                      double angularStepDegrees, ESrpChannels channels);

    /// @brief Load schema 1/2 OBJ/YAML optical families, preserving exact source record order.
    /// @param descriptorPath YAML file; relative mesh paths resolve against its parent directory.
    /// @param shadow Quadrature and self-shadow policy.
    /// @return Validated model in SI, with checked mesh SHA-256 and explicit rear-family mappings.
    /// @throws std::runtime_error For malformed files, mismatched identity or invalid assignments.
    /// @throws std::invalid_argument For invalid numerical geometry or optical properties.
    CSrpPanelModel LoadSrpPanelModel(const std::filesystem::path& descriptorPath,
                                    SSrpShadowOptions shadow = {});
} // namespace simulation_gears
