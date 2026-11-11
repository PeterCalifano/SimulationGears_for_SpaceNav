/// @file CSrpResponseLut.cpp
/// @brief Validate full-sphere response invariants once, before runtime use.
#include "CSrpResponseLut.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace simulation_gears
{
    namespace
    {
        /// @brief Require a finite, increasing, uniform axis with exact endpoints and at least three nodes.
        void ValidateAxis(const std::vector<double>& axis, double lower, double upper)
        {
            if (axis.size() < 3 || !std::equal_to<double>{}(axis.front(), lower) ||
                !std::equal_to<double>{}(axis.back(), upper))
            {
                throw std::invalid_argument("SRP axes must cover the complete sphere including endpoints");
            }
            const double step = (upper - lower) / static_cast<double>(axis.size() - 1);
            for (std::size_t index = 0; index < axis.size(); ++index)
            {
                const double expected = lower + static_cast<double>(index) * step;
                if (!std::isfinite(axis[index]) || std::abs(axis[index] - expected) > 1e-10 ||
                    (index > 0 && axis[index] <= axis[index - 1]))
                {
                    throw std::invalid_argument("SRP axes must be finite, increasing and uniform");
                }
            }
        }
    } // namespace

    CSrpResponseLut::CSrpResponseLut(std::vector<double> azimuthDegrees,
                                    std::vector<double> elevationDegrees, ESrpChannels channels,
                                    std::vector<double> samples, double referenceAreaSquareMetres,
                                    SSrpLutProvenance provenance)
        : azimuthDegrees_(std::move(azimuthDegrees)), elevationDegrees_(std::move(elevationDegrees)),
          channels_(channels), samples_(std::move(samples)),
          referenceAreaSquareMetres_(referenceAreaSquareMetres), provenance_(std::move(provenance))
    {
        // Validate the axes, channels, reference area and shadow provenance
        ValidateAxis(azimuthDegrees_, -180.0, 180.0);
        ValidateAxis(elevationDegrees_, -90.0, 90.0);
        if (channels_ != ESrpChannels::Scalar && channels_ != ESrpChannels::ScalarTransverse)
        {
            throw std::invalid_argument("SRP channel selection must be scalar or scalar-transverse");
        }
        if (!std::isfinite(referenceAreaSquareMetres_) || referenceAreaSquareMetres_ <= 0.0)
        {
            throw std::invalid_argument("SRP reference area must be positive and finite [m^2]");
        }
        if (!std::isfinite(provenance_.rayOffsetMetres) || provenance_.rayOffsetMetres < 0.0 ||
            (provenance_.selfShadowing &&
             (provenance_.shadowSamples == 0 || provenance_.rayOffsetMetres <= 0.0)))
        {
            throw std::invalid_argument("SRP shadow provenance requires samples and a positive ray offset");
        }

        // Check products before indexing user-sized buffers.
        const auto channelCount = static_cast<std::size_t>(channels_);
        const auto elevationCount = elevationDegrees_.size();
        if (elevationCount > std::numeric_limits<std::size_t>::max() / channelCount ||
            azimuthDegrees_.size() > std::numeric_limits<std::size_t>::max() /
                                     (elevationCount * channelCount) ||
            samples_.size() != azimuthDegrees_.size() * elevationCount * channelCount)
        {
            throw std::invalid_argument("SRP sample count does not match axes and channels");
        }
        for (double sample : samples_)
        {
            if (!std::isfinite(sample))
            {
                throw std::invalid_argument("SRP samples must be finite");
            }
        }

        // Check each node: nonnegative Cr, exact seam and poles, transverse force orthogonal to direction
        const double radiansPerDegree = std::numbers::pi / 180.0;
        for (std::size_t azimuth = 0; azimuth < azimuthDegrees_.size(); ++azimuth)
        {
            for (std::size_t elevation = 0; elevation < elevationCount; ++elevation)
            {
                const std::size_t offset = (azimuth * elevationCount + elevation) * channelCount;
                if (samples_[offset] < 0.0)
                {
                    throw std::invalid_argument("SRP Cr samples must be nonnegative");
                }

                // Constant poles and the duplicated seam are exact storage invariants.
                if (azimuth + 1 == azimuthDegrees_.size() || elevation == 0 ||
                    elevation + 1 == elevationCount)
                {
                    for (std::size_t channel = 0; channel < channelCount; ++channel)
                    {
                        if (!std::equal_to<double>{}(samples_[offset + channel],
                                                     samples_[elevation * channelCount + channel]))
                        {
                            throw std::invalid_argument("SRP seam and pole samples must agree exactly");
                        }
                    }
                }

                if (channels_ == ESrpChannels::ScalarTransverse)
                {
                    const double longitude = azimuthDegrees_[azimuth] * radiansPerDegree;
                    const double latitude = elevationDegrees_[elevation] * radiansPerDegree;
                    const Eigen::Vector3d direction(std::cos(latitude) * std::cos(longitude),
                                                    std::cos(latitude) * std::sin(longitude),
                                                    std::sin(latitude));
                    const Eigen::Map<const Eigen::Vector3d> transverse(samples_.data() + offset + 1);
                    if (std::abs(direction.dot(transverse)) >
                        1e-10 * std::max(1.0, transverse.cwiseAbs().maxCoeff()))
                    {
                        throw std::invalid_argument("SRP transverse nodes must be orthogonal to direction");
                    }
                }
            }
        }
    }
} // namespace simulation_gears
