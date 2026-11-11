/// @file testSrpPanelModel.cpp
/// @brief Independent panel limits, optical sides, shadowing and analytic direction partials.
#include "srp_test_utils.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <limits>
#include <sstream>

using namespace simulation_gears;

TEST_CASE("SRP panel absorption diffuse and mirror limits", "[srp][panel]")
{
    const SSrpShadowOptions unshadowed{ESrpShadowing::Disabled};
    const CSrpPanelModel absorber({srp_test::Triangle()}, {{0, ESrpOpticalSide::Front, 0, 0}}, unshadowed);
    const CSrpPanelModel mirror({srp_test::Triangle()}, {{0, ESrpOpticalSide::Front, 0, 1}}, unshadowed);
    const CSrpPanelModel diffuse({srp_test::Triangle()}, {{0, ESrpOpticalSide::Front, 1, 0}}, unshadowed);
    REQUIRE(absorber.evaluate(Eigen::Vector3d::UnitZ()).forcePerPressure.z() == Catch::Approx(-0.5));
    REQUIRE(mirror.evaluate(Eigen::Vector3d::UnitZ()).forcePerPressure.z() == Catch::Approx(-1.0));
    REQUIRE(diffuse.evaluate(Eigen::Vector3d::UnitZ()).forcePerPressure.z() == Catch::Approx(-5.0 / 6.0));
    REQUIRE(absorber.evaluate(-Eigen::Vector3d::UnitZ()).forcePerPressure.isZero());
    REQUIRE_FALSE(absorber.evaluate(Eigen::Vector3d::UnitX()).regular);

    const Eigen::Vector3d direction = Eigen::Vector3d(0.3, -0.4, 0.8).normalized();
    REQUIRE((absorber.evaluate(direction).forcePerPressure + 0.5 * direction.z() * direction).norm() < 1e-15);
    REQUIRE((mirror.evaluate(direction).forcePerPressure +
             direction.z() * direction.z() * Eigen::Vector3d::UnitZ()).norm() < 1e-15);
    const auto response = absorber.evaluate(direction);
    REQUIRE((response.torquePerPressure - Eigen::Vector3d(1.0 / 3, 1.0 / 3, 0).cross(
              response.forcePerPressure)).norm() < 1e-15);
}

TEST_CASE("SRP panel analytic Jacobian includes normalization and frozen visibility", "[srp][jacobian]")
{
    const CSrpPanelModel panel({srp_test::Triangle()}, {{0, ESrpOpticalSide::Front, 0.2, 0.6}});
    const Eigen::Vector3d query(0.3, -0.5, 1.7);
    const std::array<double, 1> fractions{0.375};
    const auto response = panel.evaluate(query, fractions);
    Eigen::Matrix3d numerical;
    const double step = 1e-6;
    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
        Eigen::Vector3d offset = Eigen::Vector3d::Zero();
        offset[axis] = step;
        numerical.col(axis) = (panel.evaluate(query + offset, fractions).forcePerPressure -
                               panel.evaluate(query - offset, fractions).forcePerPressure) / (2 * step);
    }
    REQUIRE((response.directionJacobian - numerical).cwiseAbs().maxCoeff() < 1e-10);
    REQUIRE((response.directionJacobian * query).norm() < 1e-15);
    for (double scale : {1e-200, 1e200})
    {
        const auto scaled = panel.evaluate(scale * query, fractions);
        REQUIRE((scaled.forcePerPressure - response.forcePerPressure).norm() < 1e-15);
        REQUIRE((scale * scaled.directionJacobian - response.directionJacobian).norm() < 1e-14);
    }
}

TEST_CASE("SRP shadow samples respect source exclusion opaque sides and rear optics", "[srp][shadow]")
{
    const std::vector<SSrpOpticalFace> faces{{0, ESrpOpticalSide::Front, 0, 0},
                                          {0, ESrpOpticalSide::Rear, 0, 1}};
    for (unsigned int level : {3U, 4U, 5U})
    {
        const CSrpPanelModel isolated({srp_test::Triangle()}, faces,
                                      {ESrpShadowing::OpaqueTriangles, level});
        REQUIRE(isolated.visibility(Eigen::Vector3d::UnitZ())[0] == Catch::Approx(1.0));
        REQUIRE(isolated.evaluate(-Eigen::Vector3d::UnitZ()).forcePerPressure.z() == Catch::Approx(1.0));
        auto occluder = srp_test::Triangle(1.0);
        std::swap(occluder[1], occluder[2]);
        const CSrpPanelModel occluded({srp_test::Triangle(), occluder}, faces,
                                      {ESrpShadowing::OpaqueTriangles, level});
        REQUIRE(occluded.visibility(Eigen::Vector3d::UnitZ())[0] == Catch::Approx(0.0));
        REQUIRE(occluded.evaluate(Eigen::Vector3d::UnitZ()).forcePerPressure.isZero());
        REQUIRE(occluded.visibility(-Eigen::Vector3d::UnitZ())[1] == Catch::Approx(1.0));
    }
    // One homothetic half-size triangle blocks exactly one quarter of equal-area samples.
    auto partialOccluder = srp_test::Triangle(1.0);
    partialOccluder[1].x() = 0.5;
    partialOccluder[2].y() = 0.5;
    const CSrpPanelModel partial({srp_test::Triangle(), partialOccluder}, faces);
    REQUIRE(partial.visibility(Eigen::Vector3d::UnitZ())[0] == Catch::Approx(0.75));
}

TEST_CASE("SRP panel rejects invalid boundary data", "[srp][validation]")
{
    REQUIRE_THROWS_AS(CSrpPanelModel({}, {}), std::invalid_argument);
    REQUIRE_THROWS_AS(CSrpPanelModel({srp_test::Triangle()}, {{0, ESrpOpticalSide::Front, 0.5, 0.6}}),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(CSrpPanelModel({srp_test::Triangle()}, {{1, ESrpOpticalSide::Front, 0, 0}}),
                      std::invalid_argument);
    const CSrpPanelModel panel({srp_test::Triangle()}, {{0, ESrpOpticalSide::Front, 0, 0}});
    REQUIRE_THROWS_AS(panel.evaluate(Eigen::Vector3d::Zero()), std::invalid_argument);
    REQUIRE_THROWS_AS(panel.evaluate(Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())),
                      std::invalid_argument);
    const std::array<double, 1> invalidVisibility{-0.1};
    REQUIRE_THROWS_AS(panel.evaluate(Eigen::Vector3d::UnitZ(), invalidVisibility), std::invalid_argument);
}

TEST_CASE("SRP panel matches pinned MATLAB force Jacobian and torque oracle", "[srp][parity]")
{
    const SrpTriangle yzTriangle{Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};
    const CSrpPanelModel model({srp_test::Triangle(), yzTriangle},
        {{0, ESrpOpticalSide::Front, 0.2, 0.6}, {1, ESrpOpticalSide::Front, 0.1, 0.3}},
        {ESrpShadowing::Disabled});
    const std::array<double, 2> visibility{0.375, 0.625};
    std::ifstream input(std::filesystem::path(SRP_FIXTURE_DIRECTORY) / "matlab_panel.csv");
    REQUIRE(input.good());
    std::string line;
    std::size_t rows = 0;
    while (std::getline(input, line))
    {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream row(line);
        Eigen::Vector3d query, force, torque;
        Eigen::Matrix3d jacobian;
        // Read one row: query, force, column-major Jacobian, torque
        for (auto& value : query)
        {
            REQUIRE(static_cast<bool>(row >> value));
        }
        for (auto& value : force)
        {
            REQUIRE(static_cast<bool>(row >> value));
        }
        for (Eigen::Index index = 0; index < jacobian.size(); ++index)
        {
            REQUIRE(static_cast<bool>(row >> jacobian.data()[index]));
        }
        for (auto& value : torque)
        {
            REQUIRE(static_cast<bool>(row >> value));
        }

        // Compare the native response with the MATLAB row
        const auto response = model.evaluate(query, visibility);
        REQUIRE((response.forcePerPressure - force).norm() < 2e-14);
        REQUIRE((response.directionJacobian - jacobian).norm() < 2e-14);
        REQUIRE((response.torquePerPressure - torque).norm() < 2e-14);
        ++rows;
    }
    REQUIRE(rows == 5);

    // Compare the native table with the MATLAB-built table sample by sample
    const auto reference = LoadSrpResponseLut(std::filesystem::path(SRP_FIXTURE_DIRECTORY) /
                                             "matlab_response.json");
    const auto native = BuildSrpResponseLut(model, 0.73, 30, ESrpChannels::ScalarTransverse);
    REQUIRE(reference.samples().size() == native.samples().size());
    for (std::size_t index = 0; index < reference.samples().size(); ++index)
    {
        REQUIRE(std::abs(reference.samples()[index] - native.samples()[index]) < 2e-14);
    }
}
