/// @file SrpLutJson.cpp
/// @brief JSON ingress/egress isolated from SRP numerical kernels.
#include "CSrpResponseLut.h"

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/prettywriter.h>
#include <fstream>
#include <stdexcept>

namespace simulation_gears
{
    namespace
    {
        constexpr const char* schema = "SimulationGears.SrpResponseLut.v1";

        /// @brief Return a required member of a JSON object.
        const rapidjson::Value& Required(const rapidjson::Value& object, const char* key)
        {
            if (!object.IsObject() || !object.HasMember(key))
            {
                throw std::runtime_error(std::string("SRP JSON missing field: ") + key);
            }
            return object[key];
        }

        /// @brief Return a required string member.
        std::string String(const rapidjson::Value& object, const char* key)
        {
            const auto& value = Required(object, key);
            if (!value.IsString())
            {
                throw std::runtime_error(std::string("SRP JSON expects string: ") + key);
            }
            return {value.GetString(), value.GetStringLength()};
        }

        /// @brief Return a required number member as double.
        double Number(const rapidjson::Value& object, const char* key)
        {
            const auto& value = Required(object, key);
            if (!value.IsNumber())
            {
                throw std::runtime_error(std::string("SRP JSON expects number: ") + key);
            }
            return value.GetDouble();
        }

        /// @brief Return a required array member of numbers.
        std::vector<double> Array(const rapidjson::Value& object, const char* key)
        {
            const auto& value = Required(object, key);
            if (!value.IsArray())
            {
                throw std::runtime_error(std::string("SRP JSON expects array: ") + key);
            }
            std::vector<double> result;
            result.reserve(value.Size());
            for (const auto& sample : value.GetArray())
            {
                if (!sample.IsNumber())
                {
                    throw std::runtime_error("SRP JSON array must contain numbers");
                }
                result.push_back(sample.GetDouble());
            }
            return result;
        }

        /// @brief Reject an object, at any depth, that repeats a key.
        void RejectDuplicateKeys(const rapidjson::Value& value)
        {
            if (value.IsObject())
            {
                for (auto member = value.MemberBegin(); member != value.MemberEnd(); ++member)
                {
                    for (auto previous = value.MemberBegin(); previous != member; ++previous)
                    {
                        if (previous->name == member->name)
                        {
                            throw std::runtime_error("SRP JSON duplicate object key");
                        }
                    }
                    RejectDuplicateKeys(member->value);
                }
            }
            else if (value.IsArray())
            {
                for (const auto& item : value.GetArray())
                {
                    RejectDuplicateKeys(item);
                }
            }
        }
    } // namespace

    CSrpResponseLut LoadSrpResponseLut(const std::filesystem::path& path)
    {
        // Parse the file at full precision and reject duplicate keys
        std::ifstream input(path);
        if (!input)
        {
            throw std::runtime_error("Cannot read SRP JSON: " + path.string());
        }
        rapidjson::IStreamWrapper stream(input);
        rapidjson::Document document;
        document.ParseStream<rapidjson::kParseFullPrecisionFlag>(stream);
        if (document.HasParseError())
        {
            throw std::runtime_error(std::string("SRP JSON parse error: ") +
                                     rapidjson::GetParseError_En(document.GetParseError()));
        }
        RejectDuplicateKeys(document);

        // Require the v1 schema, units, frame, direction and layout
        if (String(document, "schema") != schema || String(document, "axis_units") != "deg" ||
            String(document, "frame") != "SCB" ||
            String(document, "direction") != "spacecraft_to_sun" ||
            String(document, "layout") != "azimuth_elevation_channel" ||
            String(document, "area_units") != "m^2")
        {
            throw std::runtime_error("Unsupported SRP JSON schema, units, frame or layout");
        }

        // Require the channel names and units in order
        const auto& channelNames = Required(document, "channels");
        if (!channelNames.IsArray() || (channelNames.Size() != 1 && channelNames.Size() != 4))
        {
            throw std::runtime_error("SRP JSON requires Cr or Cr,Tx,Ty,Tz channels");
        }
        const char* expected[] = {"Cr", "Tx", "Ty", "Tz"};
        for (rapidjson::SizeType index = 0; index < channelNames.Size(); ++index)
        {
            if (!channelNames[index].IsString() || channelNames[index] != expected[index])
            {
                throw std::runtime_error("SRP JSON channel order must be Cr,Tx,Ty,Tz");
            }
        }
        const auto& units = Required(document, "channel_units");
        if (!units.IsArray() || units.Size() != channelNames.Size())
        {
            throw std::runtime_error("SRP JSON requires one unit per channel");
        }
        for (rapidjson::SizeType index = 0; index < units.Size(); ++index)
        {
            if (!units[index].IsString() || units[index] != (index == 0 ? "1" : "m^2"))
            {
                throw std::runtime_error("SRP JSON channel units must be 1,m^2,m^2,m^2");
            }
        }

        // Read the provenance, then let the table validate the numbers
        const auto& source = Required(document, "provenance");
        SSrpLutProvenance provenance;
        provenance.geometrySha256 = String(source, "geometry_sha256");
        provenance.descriptorSha256 = String(source, "descriptor_sha256");
        provenance.generator = String(source, "generator");
        if (!Required(source, "self_shadowing").IsBool() ||
            !Required(source, "shadow_samples").IsUint())
        {
            throw std::runtime_error("SRP JSON invalid shadow provenance");
        }
        provenance.selfShadowing = source["self_shadowing"].GetBool();
        provenance.shadowSamples = source["shadow_samples"].GetUint();
        provenance.rayOffsetMetres = Number(source, "ray_offset_m");
        return CSrpResponseLut(Array(document, "azimuth"), Array(document, "elevation"),
                              static_cast<ESrpChannels>(channelNames.Size()), Array(document, "samples"),
                              Number(document, "reference_area_m2"), std::move(provenance));
    }

    void SaveSrpResponseLut(const CSrpResponseLut& lut, const std::filesystem::path& path)
    {
        std::ofstream output(path);
        if (!output)
        {
            throw std::runtime_error("Cannot write SRP JSON: " + path.string());
        }
        rapidjson::OStreamWrapper stream(output);
        rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(stream);
        writer.StartObject();

        // Define the key-value writers
        const auto writeString = [&writer](const char* key, const std::string& value)
        {
            writer.Key(key);
            writer.String(value.data(), static_cast<rapidjson::SizeType>(value.size()));
        };
        const auto writeArray = [&writer](const char* key, const std::vector<double>& values)
        {
            writer.Key(key);
            writer.StartArray();
            for (double value : values)
            {
                writer.Double(value);
            }
            writer.EndArray();
        };

        // Write the schema, units, frame, direction and layout
        writeString("schema", schema);
        writeString("axis_units", "deg");
        writeString("frame", "SCB");
        writeString("direction", "spacecraft_to_sun");
        writeString("layout", "azimuth_elevation_channel");
        writeString("area_units", "m^2");

        // Write the channel names and units
        writer.Key("channels");
        writer.StartArray();
        writer.String("Cr");
        if (lut.channels() == ESrpChannels::ScalarTransverse)
        {
            writer.String("Tx");
            writer.String("Ty");
            writer.String("Tz");
        }
        writer.EndArray();
        writer.Key("channel_units");
        writer.StartArray();
        writer.String("1");
        if (lut.channels() == ESrpChannels::ScalarTransverse)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                writer.String("m^2");
            }
        }
        writer.EndArray();

        // Write the reference area, the axes and the samples
        writer.Key("reference_area_m2");
        writer.Double(lut.referenceAreaSquareMetres());
        writeArray("azimuth", lut.azimuthDegrees());
        writeArray("elevation", lut.elevationDegrees());
        writeArray("samples", lut.samples());

        // Write the provenance
        writer.Key("provenance");
        writer.StartObject();
        writeString("geometry_sha256", lut.provenance().geometrySha256);
        writeString("descriptor_sha256", lut.provenance().descriptorSha256);
        writeString("generator", lut.provenance().generator);
        writer.Key("self_shadowing");
        writer.Bool(lut.provenance().selfShadowing);
        writer.Key("shadow_samples");
        writer.Uint(lut.provenance().shadowSamples);
        writer.Key("ray_offset_m");
        writer.Double(lut.provenance().rayOffsetMetres);
        writer.EndObject();
        writer.EndObject();

        // Flush and report a failed write
        output.flush();
        if (!output)
        {
            throw std::runtime_error("Failed writing SRP JSON: " + path.string());
        }
    }
} // namespace simulation_gears
