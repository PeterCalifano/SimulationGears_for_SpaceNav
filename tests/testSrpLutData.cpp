/// @file testSrpLutData.cpp
/// @brief Portable LUT shape, ordering, lifetime, ingress failures and optical descriptor tests.
#include "srp_test_utils.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <limits>
#include <sstream>

using namespace simulation_gears;

namespace
{
    CSrpResponseLut Example(ESrpChannels channels = ESrpChannels::ScalarTransverse)
    {
        const CSrpPanelModel model({srp_test::Triangle()}, {{0, ESrpOpticalSide::Front, 0.2, 0.6}},
                                   {ESrpShadowing::Disabled});
        return BuildSrpResponseLut(model, 0.73, 30.0, channels);
    }

    std::string Read(const std::filesystem::path& path)
    {
        std::ifstream input(path);
        REQUIRE(input.good());
        std::ostringstream contents;
        contents << input.rdbuf();
        return contents.str();
    }

    void Replace(std::string& text, const std::string& before, const std::string& after)
    {
        const auto position = text.find(before);
        REQUIRE(position != std::string::npos);
        text.replace(position, before.size(), after);
    }
}

TEST_CASE("SRP JSON round trips exact samples and owns array storage", "[srp][lut][json]")
{
    srp_test::CTemporaryDirectory scratch;
    for (auto channels : {ESrpChannels::Scalar, ESrpChannels::ScalarTransverse})
    {
        const auto original = Example(channels);
        SaveSrpResponseLut(original, scratch.file("lut.json"));
        const auto restored = LoadSrpResponseLut(scratch.file("lut.json"));
        REQUIRE(restored.samples() == original.samples());
        REQUIRE(restored.azimuthDegrees() == original.azimuthDegrees());
        REQUIRE(restored.elevationDegrees() == original.elevationDegrees());
        REQUIRE(restored.referenceAreaSquareMetres() == Catch::Approx(0.73));
        REQUIRE(restored.channels() == channels);
        REQUIRE(restored.provenance().generator == original.provenance().generator);
    }
    auto samples = std::vector<double>(9, 1.2345678901234567);
    auto axis = std::vector<double>{-180, 0, 180};
    const CSrpResponseLut copied(axis, {-90, 0, 90}, ESrpChannels::Scalar, samples, 1);
    samples[0] = 999;
    axis[0] = 999;
    REQUIRE(copied.samples()[0] == Catch::Approx(1.2345678901234567));
    REQUIRE(copied.azimuthDegrees()[0] == Catch::Approx(-180));
}

TEST_CASE("SRP table node order and transverse decomposition match direct panels", "[srp][lut]")
{
    const CSrpPanelModel model({srp_test::Triangle()}, {{0, ESrpOpticalSide::Front, 0.2, 0.6}},
                               {ESrpShadowing::Disabled});
    const auto lut = BuildSrpResponseLut(model, 0.73, 30, ESrpChannels::ScalarTransverse);
    const auto elevationCount = lut.elevationDegrees().size();
    // Rebuild each node's force from Cr and the transverse channels and compare with the panels
    for (std::size_t longitude = 0; longitude < lut.azimuthDegrees().size(); ++longitude)
    {
        for (std::size_t latitude = 0; latitude < elevationCount; ++latitude)
        {
            const double azimuth = lut.azimuthDegrees()[longitude] * std::acos(-1.0) / 180;
            const double elevation = lut.elevationDegrees()[latitude] * std::acos(-1.0) / 180;
            const Eigen::Vector3d direction(std::cos(elevation) * std::cos(azimuth),
                                            std::cos(elevation) * std::sin(azimuth), std::sin(elevation));
            const auto offset = (longitude * elevationCount + latitude) * 4;
            const Eigen::Map<const Eigen::Vector3d> transverse(lut.samples().data() + offset + 1);
            const Eigen::Vector3d force = -0.73 * lut.samples()[offset] * direction + transverse;
            REQUIRE((force - model.evaluate(direction).forcePerPressure).norm() < 2e-15);
        }
    }
}

TEST_CASE("SRP LUT input validation rejects broken boundaries and data", "[srp][lut][validation]")
{
    const auto valid = Example();
    auto values = valid.samples();
    values[0] = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_THROWS_AS(CSrpResponseLut(valid.azimuthDegrees(), valid.elevationDegrees(), valid.channels(),
                                     values, 1), std::invalid_argument);
    values = valid.samples();
    values[4] = -1;
    REQUIRE_THROWS_AS(CSrpResponseLut(valid.azimuthDegrees(), valid.elevationDegrees(), valid.channels(),
                                     values, 1), std::invalid_argument);
    values = valid.samples();
    values[values.size() - 4] += 1;
    REQUIRE_THROWS_AS(CSrpResponseLut(valid.azimuthDegrees(), valid.elevationDegrees(), valid.channels(),
                                     values, 1), std::invalid_argument);
    values = valid.samples();
    values[(2 * valid.elevationDegrees().size() + 4) * 4 + 3] += 1;
    REQUIRE_THROWS_AS(CSrpResponseLut(valid.azimuthDegrees(), valid.elevationDegrees(), valid.channels(),
                                     values, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(CSrpResponseLut({-180, 0, 179}, {-90, 0, 90}, ESrpChannels::Scalar,
                                     std::vector<double>(9, 1), 1), std::invalid_argument);
    REQUIRE_THROWS_AS(CSrpResponseLut({-180, 0, 180}, {-90, 0, 90}, ESrpChannels::Scalar,
                                     std::vector<double>(8, 1), 1), std::invalid_argument);
    REQUIRE_THROWS_AS(CSrpResponseLut({-180, 0, 180}, {-90, 0, 90}, ESrpChannels::Scalar,
                                     std::vector<double>(9, 1), 0), std::invalid_argument);
}

TEST_CASE("SRP axis tolerance follows the shared portable schema", "[srp][lut][validation]")
{
    for (double deviation : {1e-11, 1e-8})
    {
        const std::vector<double> azimuth{-180, -90 + deviation, 0, 90, 180};
        if (deviation < 1e-10)
        {
            REQUIRE_NOTHROW(CSrpResponseLut(azimuth, {-90, 0, 90}, ESrpChannels::Scalar,
                                            std::vector<double>(15, 1), 1));
        }
        else
        {
            REQUIRE_THROWS_AS(CSrpResponseLut(azimuth, {-90, 0, 90}, ESrpChannels::Scalar,
                                              std::vector<double>(15, 1), 1), std::invalid_argument);
        }
    }
}

TEST_CASE("SRP JSON rejects unsupported contracts and malformed data", "[srp][json][validation]")
{
    srp_test::CTemporaryDirectory scratch;
    const auto path = scratch.file("lut.json");
    SaveSrpResponseLut(Example(), path);
    const auto validText = Read(path);
    for (const auto& [before, after] : std::vector<std::pair<std::string, std::string>>{
            {"SrpResponseLut.v1", "SrpResponseLut.v2"}, {"\"deg\"", "\"rad\""},
            {"\"Tx\"", "\"Ty\""}, {"\"m^2\"", "\"km^2\""},
            {"\"SCB\"", "\"IN\""}, {"\"samples\"", "\"absent\""},
            {"{", "{\"schema\":\"duplicate\","}})
    {
        auto malformed = validText;
        Replace(malformed, before, after);
        srp_test::Write(path, malformed);
        REQUIRE_THROWS(LoadSrpResponseLut(path));
    }
    srp_test::Write(path, "{not valid JSON");
    REQUIRE_THROWS(LoadSrpResponseLut(path));
    REQUIRE_THROWS(LoadSrpResponseLut(scratch.file("missing.json")));
}

TEST_CASE("SRP optical descriptor preserves source identity and explicit rear families", "[srp][yaml]")
{
    const auto fixtures = std::filesystem::path(SRP_FIXTURE_DIRECTORY);
    const auto descriptor = fixtures / "two_triangles.srp.yml";
    const auto model = LoadSrpPanelModel(descriptor, {ESrpShadowing::Disabled});
    REQUIRE(model.triangleCount() == 2);
    REQUIRE(model.opticalFaceCount() == 3);
    REQUIRE(model.evaluate(Eigen::Vector3d::UnitZ()).forcePerPressure.z() == Catch::Approx(-1.5));
    REQUIRE(model.evaluate(-Eigen::Vector3d::UnitZ()).forcePerPressure.z() == Catch::Approx(1.0));
    REQUIRE(model.provenance().geometrySha256 ==
            "76ef7517546d82a59b7217ab5f19417c07f059161e82997ceb3bbd3ba212dfc7");
    REQUIRE(model.provenance().descriptorSha256.size() == 64);

    srp_test::CTemporaryDirectory scratch;
    std::filesystem::copy_file(fixtures / "two_triangles.obj", scratch.file("two_triangles.obj"));
    const auto validText = Read(descriptor);
    for (const auto& [before, after] : std::vector<std::pair<std::string, std::string>>{
            {"schema_version: 2", "schema_version: 99"}, {"76ef7517", "00000000"},
            {"[[2, 2]]", "[[1, 2]]"}, {"length_unit: m", "length_unit: cm"},
            {"back_reflectivity_family: Mirror", "back_reflectivity_family: Missing"},
            {"specular: 1.0", "specular: 1.1"}, {"triangle_count: 2", "triangle_count: 1"},
            {"schema_version: 2", "schema_version: 2\nschema_version: 2"}})
    {
        auto malformed = validText;
        Replace(malformed, before, after);
        srp_test::Write(scratch.file("bad.yml"), malformed);
        REQUIRE_THROWS(LoadSrpPanelModel(scratch.file("bad.yml")));
    }
    srp_test::Write(scratch.file("bad.yml"), "[a: b\n}");
    REQUIRE_THROWS(LoadSrpPanelModel(scratch.file("bad.yml")));
}
