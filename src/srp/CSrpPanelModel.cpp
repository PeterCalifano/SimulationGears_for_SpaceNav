/// @file CSrpPanelModel.cpp
/// @brief Panel force law, frozen-visibility partials and equal-area shadow quadrature.
#include "CSrpPanelModel.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <set>
#include <stdexcept>
#include <utility>

namespace simulation_gears
{
    namespace
    {
        /// @brief Return the unit vector of a finite nonzero direction, scaling first to avoid overflow.
        Eigen::Vector3d Normalize(const Eigen::Vector3d& vector)
        {
            const double scale = vector.cwiseAbs().maxCoeff();
            if (!vector.allFinite() || scale <= 0.0)
            {
                throw std::invalid_argument("SRP Sun direction must be finite and nonzero");
            }
            return (vector / scale).normalized();
        }

        /// @brief Return the centroids of the 4^level equal-area children of a triangle.
        std::vector<Eigen::Vector3d> EqualAreaCentroids(const SrpTriangle& triangle, unsigned int level)
        {
            // Split every triangle into four by its edge midpoints, level times
            std::vector<SrpTriangle> children{triangle};
            for (unsigned int depth = 0; depth < level; ++depth)
            {
                std::vector<SrpTriangle> next;
                next.reserve(children.size() * 4);
                for (const auto& parent : children)
                {
                    const Eigen::Vector3d first = 0.5 * (parent[0] + parent[1]);
                    const Eigen::Vector3d second = 0.5 * (parent[1] + parent[2]);
                    const Eigen::Vector3d third = 0.5 * (parent[2] + parent[0]);
                    next.push_back({parent[0], first, third});
                    next.push_back({first, parent[1], second});
                    next.push_back({third, second, parent[2]});
                    next.push_back({first, second, third});
                }
                children = std::move(next);
            }

            // Return one centroid per child
            std::vector<Eigen::Vector3d> centroids;
            centroids.reserve(children.size());
            for (const auto& child : children)
            {
                centroids.push_back((child[0] + child[1] + child[2]) / 3.0);
            }
            return centroids;
        }

        /// @brief Tell whether a ray hits a triangle beyond a minimum distance, from either side.
        bool Intersects(const SrpTriangle& triangle, const Eigen::Vector3d& origin,
                        const Eigen::Vector3d& direction, double minimumDistance)
        {
            // Two-sided Moller-Trumbore test; source exclusion happens before this call.
            const Eigen::Vector3d firstEdge = triangle[1] - triangle[0];
            const Eigen::Vector3d secondEdge = triangle[2] - triangle[0];
            const Eigen::Vector3d perpendicular = direction.cross(secondEdge);
            const double determinant = firstEdge.dot(perpendicular);
            if (std::abs(determinant) < 2.0 * std::numeric_limits<double>::epsilon())
            {
                return false;
            }

            // Reject hits outside the triangle, then hits closer than the minimum distance
            const Eigen::Vector3d relative = origin - triangle[0];
            const double firstWeight = relative.dot(perpendicular) / determinant;
            if (firstWeight < 0.0 || firstWeight > 1.0)
            {
                return false;
            }
            const Eigen::Vector3d cross = relative.cross(firstEdge);
            const double secondWeight = direction.dot(cross) / determinant;
            if (secondWeight < 0.0 || firstWeight + secondWeight > 1.0)
            {
                return false;
            }
            return secondEdge.dot(cross) / determinant > minimumDistance;
        }
    } // namespace

    CSrpPanelModel::CSrpPanelModel(std::vector<SrpTriangle> triangles,
                                  std::vector<SSrpOpticalFace> opticalFaces,
                                  SSrpShadowOptions shadow, SSrpLutProvenance provenance)
        : triangles_(std::move(triangles)), shadow_(shadow), provenance_(std::move(provenance))
    {
        // Validate the inputs and the shadow options
        if (triangles_.empty() || opticalFaces.empty())
        {
            throw std::invalid_argument("SRP model requires source triangles and optical faces");
        }
        if (shadow_.subdivisionLevel > 5 || !std::isfinite(shadow_.rayOffsetMetres) ||
            shadow_.rayOffsetMetres <= 0.0 ||
            (shadow_.mode != ESrpShadowing::Disabled && shadow_.mode != ESrpShadowing::OpaqueTriangles))
        {
            throw std::invalid_argument("SRP shadow preparation requires level 0..5 and positive offset");
        }

        // Require finite, nondegenerate source triangles
        for (const auto& triangle : triangles_)
        {
            const Eigen::Vector3d cross = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]);
            if (!triangle[0].allFinite() || !triangle[1].allFinite() || !triangle[2].allFinite() ||
                !cross.allFinite() || cross.stableNorm() <= 0.0 || !std::isfinite(cross.stableNorm()))
            {
                throw std::invalid_argument("SRP triangles must be finite and nondegenerate");
            }
        }

        // Prepare each optical face once: unique source and side, physical optics, signed normal
        std::set<std::pair<std::size_t, ESrpOpticalSide>> assigned;
        opticalFaces_.reserve(opticalFaces.size());
        for (const auto& face : opticalFaces)
        {
            if (face.sourceTriangle >= triangles_.size() ||
                (face.side != ESrpOpticalSide::Front && face.side != ESrpOpticalSide::Rear) ||
                !std::isfinite(face.diffuse) || !std::isfinite(face.specular) ||
                face.diffuse < 0.0 || face.specular < 0.0 || face.diffuse + face.specular > 1.0 ||
                !assigned.emplace(face.sourceTriangle, face.side).second)
            {
                throw std::invalid_argument("SRP optical faces require unique source/side and physical optics");
            }
            const auto& triangle = triangles_[face.sourceTriangle];
            const Eigen::Vector3d cross = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]);
            const double doubledArea = cross.stableNorm();
            const double sign = face.side == ESrpOpticalSide::Front ? 1.0 : -1.0;
            opticalFaces_.push_back({face, sign * cross / doubledArea,
                                     (triangle[0] + triangle[1] + triangle[2]) / 3.0, 0.5 * doubledArea});
        }

        // Preparation owns all quadrature allocation; runtime visibility only reads samples.
        if (shadow_.mode == ESrpShadowing::OpaqueTriangles)
        {
            for (const auto& triangle : triangles_)
            {
                samples_.push_back(EqualAreaCentroids(triangle, shadow_.subdivisionLevel));
            }
        }

        // Record the shadow policy in the provenance
        provenance_.selfShadowing = shadow_.mode == ESrpShadowing::OpaqueTriangles;
        provenance_.shadowSamples = 1U << (2U * shadow_.subdivisionLevel);
        provenance_.rayOffsetMetres = shadow_.rayOffsetMetres;
    }

    double CSrpPanelModel::visibleFraction(std::size_t sourceTriangle,
                                          const Eigen::Vector3d& direction) const
    {
        if (shadow_.mode == ESrpShadowing::Disabled)
        {
            return 1.0;
        }

        // Count the samples whose offset ray toward the Sun hits no other triangle
        std::size_t visible = 0;
        for (const auto& sample : samples_[sourceTriangle])
        {
            const Eigen::Vector3d origin = sample + shadow_.rayOffsetMetres * direction;
            bool blocked = false;
            for (std::size_t occluder = 0; occluder < triangles_.size(); ++occluder)
            {
                if (occluder != sourceTriangle &&
                    Intersects(triangles_[occluder], origin, direction, shadow_.rayOffsetMetres))
                {
                    blocked = true;
                    break;
                }
            }
            if (!blocked)
            {
                ++visible;
            }
        }
        return static_cast<double>(visible) / static_cast<double>(samples_[sourceTriangle].size());
    }

    std::vector<double> CSrpPanelModel::visibility(const Eigen::Vector3d& directionBody) const
    {
        const Eigen::Vector3d direction = Normalize(directionBody);
        std::vector<double> fractions;
        fractions.reserve(opticalFaces_.size());
        for (const auto& face : opticalFaces_)
        {
            fractions.push_back(face.normal.dot(direction) > 0.0 ?
                                visibleFraction(face.optics.sourceTriangle, direction) : 0.0);
        }
        return fractions;
    }

    SSrpPanelResponse CSrpPanelModel::evaluate(const Eigen::Vector3d& directionBody,
                                              std::span<const double> frozenVisibility) const
    {
        // Validate the direction and the optional frozen visibility fractions
        const Eigen::Vector3d direction = Normalize(directionBody);
        if (!frozenVisibility.empty() && frozenVisibility.size() != opticalFaces_.size())
        {
            throw std::invalid_argument("SRP visibility must have one fraction per optical face");
        }
        for (double fraction : frozenVisibility)
        {
            if (!std::isfinite(fraction) || fraction < 0.0 || fraction > 1.0)
            {
                throw std::invalid_argument("SRP visibility fractions must lie in [0,1]");
            }
        }

        // Sum force, torque and the unit-direction Jacobian over the illuminated faces
        SSrpPanelResponse response;
        for (std::size_t index = 0; index < opticalFaces_.size(); ++index)
        {
            const auto& face = opticalFaces_[index];
            const double incidence = face.normal.dot(direction);
            if (std::abs(incidence) < 1e-14)
            {
                response.regular = false;
            }
            if (incidence <= 0.0)
            {
                continue;
            }
            const double fraction = frozenVisibility.empty() ?
                visibleFraction(face.optics.sourceTriangle, direction) : frozenVisibility[index];
            const double illuminatedArea = face.area * fraction;
            const double diffuse = face.optics.diffuse;
            const double specular = face.optics.specular;
            const Eigen::Vector3d force = -illuminatedArea * incidence *
                (2.0 * (diffuse / 3.0 + specular * incidence) * face.normal +
                 (1.0 - specular) * direction);
            response.forcePerPressure += force;
            response.torquePerPressure += face.centre.cross(force);
            response.directionJacobian -= illuminatedArea *
                ((2.0 * diffuse / 3.0 + 4.0 * specular * incidence) *
                     face.normal * face.normal.transpose() +
                 (1.0 - specular) * (direction * face.normal.transpose() +
                                    incidence * Eigen::Matrix3d::Identity()));
        }

        // Chain through the normalization. Divide successively to avoid overflow of the original
        // query norm.
        const double scale = directionBody.cwiseAbs().maxCoeff();
        response.directionJacobian =
            ((response.directionJacobian * (Eigen::Matrix3d::Identity() -
                                            direction * direction.transpose())) /
             (directionBody / scale).norm()) / scale;
        return response;
    }

    CSrpResponseLut BuildSrpResponseLut(const CSrpPanelModel& model, double referenceAreaSquareMetres,
                                      double angularStepDegrees, ESrpChannels channels)
    {
        // Validate the area, the channels and a step that divides 180 degrees
        if (!std::isfinite(angularStepDegrees) || angularStepDegrees < 0.5 || angularStepDegrees > 90.0 ||
            !std::isfinite(referenceAreaSquareMetres) || referenceAreaSquareMetres <= 0.0 ||
            (channels != ESrpChannels::Scalar && channels != ESrpChannels::ScalarTransverse))
        {
            throw std::invalid_argument("SRP LUT requires positive area, valid channels and step in [0.5,90]");
        }
        const double intervals = std::round(180.0 / angularStepDegrees);
        if (std::abs(intervals * angularStepDegrees - 180.0) > 1e-10)
        {
            throw std::invalid_argument("SRP LUT angular step must divide 180 degrees");
        }

        // Build the full-sphere azimuth and elevation axes, endpoints included
        const auto elevationCount = static_cast<std::size_t>(intervals) + 1;
        const auto azimuthCount = 2 * (elevationCount - 1) + 1;
        const auto channelCount = static_cast<std::size_t>(channels);
        std::vector<double> azimuth(azimuthCount), elevation(elevationCount);
        for (std::size_t index = 0; index < azimuthCount; ++index)
        {
            azimuth[index] = -180.0 + 360.0 * static_cast<double>(index) /
                                      static_cast<double>(azimuthCount - 1);
        }
        for (std::size_t index = 0; index < elevationCount; ++index)
        {
            elevation[index] = -90.0 + 180.0 * static_cast<double>(index) /
                                      static_cast<double>(elevationCount - 1);
        }

        // Evaluate each node; copy the seam and pole nodes exactly from the first azimuth column
        std::vector<double> samples(azimuthCount * elevationCount * channelCount);
        for (std::size_t longitude = 0; longitude < azimuthCount; ++longitude)
        {
            for (std::size_t latitude = 0; latitude < elevationCount; ++latitude)
            {
                const auto offset = (longitude * elevationCount + latitude) * channelCount;
                if (longitude > 0 && (longitude + 1 == azimuthCount || latitude == 0 ||
                                      latitude + 1 == elevationCount))
                {
                    std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(latitude * channelCount),
                                channelCount, samples.begin() + static_cast<std::ptrdiff_t>(offset));
                    continue;
                }
                const double azimuthRadians = azimuth[longitude] * std::numbers::pi / 180.0;
                const double elevationRadians = elevation[latitude] * std::numbers::pi / 180.0;
                Eigen::Vector3d direction(std::cos(elevationRadians) * std::cos(azimuthRadians),
                                          std::cos(elevationRadians) * std::sin(azimuthRadians),
                                          std::sin(elevationRadians));
                if (latitude == 0)
                {
                    direction = -Eigen::Vector3d::UnitZ();
                }
                if (latitude + 1 == elevationCount)
                {
                    direction = Eigen::Vector3d::UnitZ();
                }

                // Store Cr from the force along the direction, then the transverse force
                const auto response = model.evaluate(direction);
                const double parallel = direction.dot(response.forcePerPressure);
                samples[offset] = std::max(0.0, -parallel / referenceAreaSquareMetres);
                if (channels == ESrpChannels::ScalarTransverse)
                {
                    Eigen::Map<Eigen::Vector3d>(samples.data() + offset + 1) =
                        response.forcePerPressure - parallel * direction;
                }
            }
        }

        // Hand the samples to the validating table with the model provenance
        auto provenance = model.provenance();
        provenance.generator = "SimulationGears native panel law v1";
        return CSrpResponseLut(std::move(azimuth), std::move(elevation), channels, std::move(samples),
                              referenceAreaSquareMetres, std::move(provenance));
    }
} // namespace simulation_gears
