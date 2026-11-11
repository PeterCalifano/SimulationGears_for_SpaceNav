/// @file SrpPanelInput.cpp
/// @brief Strict optical-family YAML/OBJ loading, mesh identity and winding audit.
// Compile the complete vendored RapidYAML implementation in exactly this translation unit.
#define RYML_SINGLE_HDR_DEFINE_NOW
#define RYML_DEFAULT_CALLBACK_USES_EXCEPTIONS
#include <ryml_all.hpp>
#include <picosha2.h>

#include "CSrpPanelModel.h"
#include <charconv>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace simulation_gears
{
    namespace
    {
        using YamlNode = ryml::ConstNodeRef;

        /// @brief Return the bytes of a file.
        std::string ReadFile(const std::filesystem::path& path)
        {
            std::ifstream input(path, std::ios::binary);
            if (!input)
            {
                throw std::runtime_error("Cannot read SRP input: " + path.string());
            }
            std::ostringstream contents;
            contents << input.rdbuf();
            if (input.bad())
            {
                throw std::runtime_error("Failed reading SRP input: " + path.string());
            }
            return contents.str();
        }

        /// @brief Return a plain YAML scalar; reject containers, aliases, anchors and tags.
        std::string Text(YamlNode node)
        {
            if (!node.has_val() || node.is_container() || node.is_val_ref() || node.has_val_anchor() ||
                node.has_val_tag())
            {
                throw std::runtime_error("SRP YAML expects a scalar without aliases/anchors");
            }
            const auto value = node.val();
            return {value.str, value.len};
        }

        /// @brief Require a mapping with exactly the given keys, each once.
        void Fields(YamlNode node, std::initializer_list<const char*> keys)
        {
            if (!node.is_map() || node.num_children() != keys.size())
            {
                throw std::runtime_error("SRP YAML mapping contains missing, duplicate or unknown fields");
            }
            std::set<std::string> seen;
            for (const auto child : node.children())
            {
                const auto key = child.key();
                if (!seen.emplace(key.str, key.len).second)
                {
                    throw std::runtime_error("SRP YAML contains a duplicate key");
                }
            }
            for (const auto* key : keys)
            {
                if (!node.has_child(ryml::to_csubstr(key)))
                {
                    throw std::runtime_error(std::string("SRP YAML missing field: ") + key);
                }
            }
        }

        /// @brief Parse the whole text as one decimal number.
        template<typename Number>
        Number ParseNumber(const std::string& text)
        {
            Number value{};
            const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
            if (result.ec != std::errc{} || result.ptr != text.data() + text.size())
            {
                throw std::runtime_error("SRP input expects a decimal number: " + text);
            }
            return value;
        }

        /// @brief Vertices in metres and zero-based triangle indices in OBJ record order.
        struct SIndexedMesh
        {
            std::vector<Eigen::Vector3d> vertices;
            std::vector<std::array<std::size_t, 3>> faces;
        };

        /// @brief Read vertices and triangular faces of an OBJ text; reject other geometry records.
        SIndexedMesh ReadObj(const std::string& source, double metresPerUnit)
        {
            SIndexedMesh mesh;
            std::istringstream stream(source);
            std::string line;
            while (std::getline(stream, line))
            {
                // Comments do not change vertex or triangle record identities.
                line = line.substr(0, line.find('#'));
                std::istringstream record(line);
                std::string kind;
                if (!(record >> kind))
                {
                    continue;
                }

                if (kind == "v")
                {
                    // Read exactly three finite coordinates and convert them to metres
                    Eigen::Vector3d vertex;
                    std::string extra;
                    if (!(record >> vertex.x() >> vertex.y() >> vertex.z()) || (record >> extra))
                    {
                        throw std::runtime_error("SRP OBJ requires exactly three coordinates per vertex");
                    }
                    vertex *= metresPerUnit;
                    if (!vertex.allFinite())
                    {
                        throw std::runtime_error("SRP OBJ vertex must be finite");
                    }
                    mesh.vertices.push_back(vertex);
                }
                else if (kind == "f")
                {
                    // Read exactly three existing one-based vertex indices
                    std::array<std::size_t, 3> face{};
                    std::string token;
                    for (auto& vertex : face)
                    {
                        if (!(record >> token))
                        {
                            throw std::runtime_error("SRP OBJ face must be triangular");
                        }
                        const auto index = ParseNumber<std::size_t>(token.substr(0, token.find('/')));
                        if (index == 0 || index > mesh.vertices.size())
                        {
                            throw std::runtime_error("SRP OBJ requires positive existing vertex indices");
                        }
                        vertex = index - 1;
                    }
                    if (record >> token)
                    {
                        throw std::runtime_error("SRP OBJ polygons are not retriangulated");
                    }
                    mesh.faces.push_back(face);
                }
                else if (kind != "vt" && kind != "vn" && kind != "o" && kind != "g" &&
                         kind != "s" && kind != "mtllib" && kind != "usemtl")
                {
                    throw std::runtime_error("Unsupported SRP OBJ record: " + kind);
                }
            }
            return mesh;
        }

        /// @brief Check manifold edges and consistent winding; return the faces of closed components.
        std::vector<bool> AuditWinding(const SIndexedMesh& mesh)
        {
            // Collect the faces that use each undirected edge, with the traversal direction
            struct SEdgeUse { std::size_t face; bool increasing; };
            std::map<std::pair<std::size_t, std::size_t>, std::vector<SEdgeUse>> edges;
            for (std::size_t face = 0; face < mesh.faces.size(); ++face)
            {
                for (std::size_t edge = 0; edge < 3; ++edge)
                {
                    const auto first = mesh.faces[face][edge];
                    const auto second = mesh.faces[face][(edge + 1) % 3];
                    if (first == second)
                    {
                        throw std::runtime_error("SRP OBJ face repeats a vertex");
                    }
                    edges[std::minmax(first, second)].push_back({face, first < second});
                }
            }

            // Mark boundary faces and link neighbours across shared edges with opposite directions
            std::vector<std::vector<std::size_t>> neighbours(mesh.faces.size());
            std::vector<bool> boundary(mesh.faces.size(), false);
            for (const auto& [edge, uses] : edges)
            {
                (void)edge;
                if (uses.size() > 2)
                {
                    throw std::runtime_error("SRP OBJ has nonmanifold edges");
                }
                if (uses.size() == 1)
                {
                    boundary[uses.front().face] = true;
                }
                else
                {
                    if (uses[0].increasing == uses[1].increasing)
                    {
                        throw std::runtime_error("SRP OBJ adjacent faces have inconsistent winding");
                    }
                    neighbours[uses[0].face].push_back(uses[1].face);
                    neighbours[uses[1].face].push_back(uses[0].face);
                }
            }

            // Closed components require positive signed volume. Never flip source normals.
            std::vector<bool> visited(mesh.faces.size(), false), closed(mesh.faces.size(), false);
            for (std::size_t seed = 0; seed < mesh.faces.size(); ++seed)
            {
                if (visited[seed])
                {
                    continue;
                }

                // Walk the connected component and accumulate six times its signed volume
                std::vector<std::size_t> component{seed};
                visited[seed] = true;
                bool isClosed = true;
                double volumeTimesSix = 0.0;
                const Eigen::Vector3d origin = mesh.vertices[mesh.faces[seed][0]];
                for (std::size_t index = 0; index < component.size(); ++index)
                {
                    const auto face = component[index];
                    isClosed = isClosed && !boundary[face];
                    const auto& vertices = mesh.faces[face];
                    volumeTimesSix += (mesh.vertices[vertices[0]] - origin).dot(
                        (mesh.vertices[vertices[1]] - origin).cross(mesh.vertices[vertices[2]] - origin));
                    for (const auto neighbour : neighbours[face])
                    {
                        if (!visited[neighbour])
                        {
                            visited[neighbour] = true;
                            component.push_back(neighbour);
                        }
                    }
                }
                if (isClosed && (!std::isfinite(volumeTimesSix) || volumeTimesSix <= 0.0))
                {
                    throw std::runtime_error("SRP OBJ closed component must have outward source winding");
                }
                for (const auto face : component)
                {
                    closed[face] = isClosed;
                }
            }
            return closed;
        }

        /// @brief One optical family: optics, optional rear family and zero-based triangle ranges.
        struct SOpticalFamily
        {
            std::string name;
            std::string rearFamily;
            double diffuse;
            double specular;
            std::vector<std::pair<std::size_t, std::size_t>> ranges;
        };
    } // namespace

    CSrpPanelModel LoadSrpPanelModel(const std::filesystem::path& descriptorPath,
                                    SSrpShadowOptions shadow)
    {
        // Parse the descriptor and check its schema version
        const std::string descriptorText = ReadFile(descriptorPath);
        // Parser exceptions stay at this boundary; no process-global callback changes are needed.
        const auto tree = ryml::parse_in_arena(ryml::to_csubstr(descriptorText));
        const auto root = tree.rootref();
        Fields(root, {"schema_version", "mesh", "families"});
        const auto version = ParseNumber<unsigned int>(Text(root["schema_version"]));
        if (version != 1 && version != 2)
        {
            throw std::runtime_error("SRP YAML schema must be version 1 or 2");
        }

        // Check the mesh declaration, its SHA-256 and its triangle count, then audit the winding
        const auto meshNode = root["mesh"];
        Fields(meshNode, {"file", "sha256", "length_unit", "triangle_count", "triangle_indexing"});
        const auto units = Text(meshNode["length_unit"]);
        if ((units != "m" && units != "km") ||
            Text(meshNode["triangle_indexing"]) != "obj_record_order_1_based")
        {
            throw std::runtime_error("SRP mesh requires m/km units and obj_record_order_1_based indexing");
        }
        const auto expectedHash = Text(meshNode["sha256"]);
        if (expectedHash.size() != 64 || expectedHash.find_first_not_of("0123456789abcdef") != std::string::npos)
        {
            throw std::runtime_error("SRP mesh sha256 must contain 64 lowercase hexadecimal digits");
        }
        const auto meshPath = descriptorPath.parent_path() / Text(meshNode["file"]);
        const std::string meshText = ReadFile(meshPath);
        if (picosha2::hash256_hex_string(meshText) != expectedHash)
        {
            throw std::runtime_error("SRP mesh bytes differ from descriptor sha256");
        }
        const auto mesh = ReadObj(meshText, units == "m" ? 1.0 : 1000.0);
        if (mesh.faces.empty() || mesh.faces.size() !=
            ParseNumber<std::size_t>(Text(meshNode["triangle_count"])))
        {
            throw std::runtime_error("SRP mesh triangle_count does not match source OBJ records");
        }
        const auto closed = AuditWinding(mesh);

        // Read the optical families: unique names, optics and one-based triangle ranges
        const auto familyNodes = root["families"];
        if (!familyNodes.is_seq() || familyNodes.num_children() == 0)
        {
            throw std::runtime_error("SRP YAML families must be a nonempty sequence");
        }
        std::vector<SOpticalFamily> families;
        std::map<std::string, std::size_t> familyByName;
        for (const auto node : familyNodes.children())
        {
            if (version == 1)
            {
                Fields(node, {"name", "triangle_ranges", "reflectivity"});
            }
            else
            {
                Fields(node, {"name", "triangle_ranges", "reflectivity", "back_reflectivity_family"});
            }
            Fields(node["reflectivity"], {"diffuse", "specular"});
            SOpticalFamily family;
            family.name = Text(node["name"]);
            family.rearFamily = version == 2 ? Text(node["back_reflectivity_family"]) : "";
            family.diffuse = ParseNumber<double>(Text(node["reflectivity"]["diffuse"]));
            family.specular = ParseNumber<double>(Text(node["reflectivity"]["specular"]));
            if (family.name.empty() || !familyByName.emplace(family.name, families.size()).second)
            {
                throw std::runtime_error("SRP optical family names must be nonempty and unique");
            }
            const auto ranges = node["triangle_ranges"];
            if (!ranges.is_seq() || ranges.num_children() == 0)
            {
                throw std::runtime_error("SRP triangle_ranges must be nonempty");
            }
            for (const auto range : ranges.children())
            {
                if (!range.is_seq() || range.num_children() != 2)
                {
                    throw std::runtime_error("SRP triangle range must contain first and last indices");
                }
                const auto first = ParseNumber<std::size_t>(Text(range[0]));
                const auto last = ParseNumber<std::size_t>(Text(range[1]));
                if (first == 0 || last < first || last > mesh.faces.size())
                {
                    throw std::runtime_error("SRP triangle range lies outside source records");
                }
                family.ranges.emplace_back(first - 1, last - 1);
            }
            families.push_back(std::move(family));
        }

        // Assign each triangle to exactly one family
        std::vector<std::size_t> assignments(mesh.faces.size(), families.size());
        for (std::size_t family = 0; family < families.size(); ++family)
        {
            for (const auto& [first, last] : families[family].ranges)
            {
                for (std::size_t triangle = first; triangle <= last; ++triangle)
                {
                    if (assignments[triangle] != families.size())
                    {
                        throw std::runtime_error("SRP optical triangle ranges overlap");
                    }
                    assignments[triangle] = family;
                }
            }
        }

        // Build the source triangles and their front optical faces in record order
        std::vector<SrpTriangle> triangles;
        std::vector<SSrpOpticalFace> opticalFaces;
        for (std::size_t index = 0; index < mesh.faces.size(); ++index)
        {
            if (assignments[index] == families.size())
            {
                throw std::runtime_error("SRP triangle lacks optics");
            }
            const auto& face = mesh.faces[index];
            triangles.push_back({mesh.vertices[face[0]], mesh.vertices[face[1]], mesh.vertices[face[2]]});
            const auto& family = families[assignments[index]];
            opticalFaces.push_back({index, ESrpOpticalSide::Front, family.diffuse, family.specular});
        }

        // Append opposing faces after every source front, matching the MATLAB payload ordering.
        for (std::size_t index = 0; index < mesh.faces.size(); ++index)
        {
            const auto& family = families[assignments[index]];
            if (family.rearFamily.empty())
            {
                continue;
            }
            if (closed[index] || !familyByName.contains(family.rearFamily))
            {
                throw std::runtime_error("SRP rear optics require an open component and an existing family");
            }
            const auto& rear = families[familyByName.at(family.rearFamily)];
            opticalFaces.push_back({index, ESrpOpticalSide::Rear, rear.diffuse, rear.specular});
        }

        // Record the mesh and descriptor identities, then prepare the model
        SSrpLutProvenance provenance;
        provenance.geometrySha256 = expectedHash;
        provenance.descriptorSha256 = picosha2::hash256_hex_string(descriptorText);
        provenance.generator = "SimulationGears optical descriptor v" + std::to_string(version);
        return CSrpPanelModel(std::move(triangles), std::move(opticalFaces), shadow, std::move(provenance));
    }
} // namespace simulation_gears
