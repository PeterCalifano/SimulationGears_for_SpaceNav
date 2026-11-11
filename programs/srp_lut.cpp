/// @file srp_lut.cpp
/// @brief Explicit native construction, validation and inspection of portable SRP LUT artifacts.
#include <sim-gears-for-space-nav/srp/CSrpPanelModel.h>
#include <charconv>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace
{
    /// @brief Parse the whole argument as one decimal number.
    double Number(std::string_view text)
    {
        double value{};
        const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
        if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
        {
            throw std::invalid_argument("Expected a decimal argument");
        }
        return value;
    }

    /// @brief Print the table shape, reference area and geometry identity.
    void Inspect(const simulation_gears::CSrpResponseLut& lut)
    {
        std::cout << "Valid SimulationGears.SrpResponseLut.v1: " << lut.azimuthDegrees().size()
                  << " azimuth x " << lut.elevationDegrees().size() << " elevation x "
                  << static_cast<int>(lut.channels()) << " channels\nReference area: "
                  << lut.referenceAreaSquareMetres() << " m^2\nGeometry SHA-256: "
                  << lut.provenance().geometrySha256 << '\n';
    }
} // namespace

/// @brief Execute an explicitly selected file operation; report actionable errors and nonzero status.
int main(int argc, char* argv[])
{
    try
    {
        using namespace simulation_gears;

        // Validate or inspect an existing artifact
        if (argc == 3 && (std::string_view(argv[1]) == "validate" || std::string_view(argv[1]) == "inspect"))
        {
            Inspect(LoadSrpResponseLut(argv[2]));
            return 0;
        }

        // Build an artifact from a descriptor with the selected channels and shadow sampling
        if (argc >= 6 && argc <= 8 && std::string_view(argv[1]) == "build")
        {
            const std::string_view channelArgument = argc >= 7 ? argv[6] : "transverse";
            if (channelArgument != "scalar" && channelArgument != "transverse")
            {
                throw std::invalid_argument("Channels must be scalar or transverse");
            }
            SSrpShadowOptions shadow;
            const std::string_view shadowArgument = argc == 8 ? argv[7] : "64";
            if (shadowArgument == "off")
            {
                shadow.mode = ESrpShadowing::Disabled;
            }
            else if (shadowArgument == "64")
            {
                shadow.subdivisionLevel = 3;
            }
            else if (shadowArgument == "256")
            {
                shadow.subdivisionLevel = 4;
            }
            else if (shadowArgument == "1024")
            {
                shadow.subdivisionLevel = 5;
            }
            else
            {
                throw std::invalid_argument("Shadow samples must be off, 64, 256 or 1024");
            }
            const auto model = LoadSrpPanelModel(argv[2], shadow);
            const auto lut = BuildSrpResponseLut(model, Number(argv[4]), Number(argv[5]),
                channelArgument == "scalar" ? ESrpChannels::Scalar : ESrpChannels::ScalarTransverse);
            SaveSrpResponseLut(lut, argv[3]);
            Inspect(lut);
            return 0;
        }

        // Print the usage for any other argument list
        std::cerr << "Usage: srp_lut build descriptor.yml output.json area_m2 step_deg "
                     "[scalar|transverse] [off|64|256|1024]\n"
                     "       srp_lut validate|inspect input.json\n";
        return 2;
    }
    catch (const std::exception& error)
    {
        std::cerr << "srp_lut: " << error.what() << '\n';
        return 1;
    }
}
