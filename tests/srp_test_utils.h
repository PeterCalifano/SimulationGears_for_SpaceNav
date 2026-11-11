/// @file srp_test_utils.h
/// @brief Isolated temporary files and small analytical geometry for SRP unit tests.
#pragma once
#include <sim-gears-for-space-nav/srp/CSrpPanelModel.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>

namespace srp_test
{
    /// @brief Test-owned scratch directory; never removes paths it did not create.
    class CTemporaryDirectory
    {
      public:
        CTemporaryDirectory()
        {
            // Try random names until one directory is created
            std::random_device random;
            for (int attempt = 0; attempt < 10; ++attempt)
            {
                path_ = std::filesystem::temp_directory_path() /
                    ("simgears-srp-test-" + std::to_string(random()) + "-" + std::to_string(random()));
                if (std::filesystem::create_directory(path_))
                {
                    return;
                }
            }
            throw std::runtime_error("Cannot create isolated SRP test directory");
        }

        ~CTemporaryDirectory()
        {
            std::error_code ignored;
            std::filesystem::remove_all(path_, ignored);
        }

        CTemporaryDirectory(const CTemporaryDirectory&) = delete;
        CTemporaryDirectory& operator=(const CTemporaryDirectory&) = delete;

        /// @brief Return a path inside the scratch directory.
        std::filesystem::path file(const std::string& name) const { return path_ / name; }

      private:
        std::filesystem::path path_;
    };

    /// @brief Return the unit right triangle in the plane z = height, wound counterclockwise about +z.
    inline simulation_gears::SrpTriangle Triangle(double height = 0.0)
    {
        return {Eigen::Vector3d(0, 0, height), Eigen::Vector3d(1, 0, height),
                Eigen::Vector3d(0, 1, height)};
    }

    /// @brief Write a text fixture.
    inline void Write(const std::filesystem::path& path, const std::string& text)
    {
        std::ofstream output(path);
        output << text;
        if (!output)
        {
            throw std::runtime_error("Cannot write SRP fixture");
        }
    }
} // namespace srp_test
