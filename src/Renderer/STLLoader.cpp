#include "STLLoader.h"
#include <fstream>
#include <stdexcept>
#include <cstdint>

#pragma pack(push, 1)
struct STLTri {
    float n[3];
    float v0[3];
    float v1[3];
    float v2[3];
    uint16_t attr;
};
#pragma pack(pop)

static glm::vec3 toVec3(const float f[3]) {
    return {f[0], f[1], f[2]};
}

STLMeshData STLLoader::LoadBinary(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Failed to open STL: " + path);
    }

    char header[80];
    in.read(header, 80);

    uint32_t triCount = 0;
    in.read(reinterpret_cast<char*>(&triCount), sizeof(uint32_t));
    if (!in) {
        throw std::runtime_error("Invalid STL header: " + path);
    }

    STLMeshData out;
    out.positions.reserve(static_cast<size_t>(triCount) * 3);

    for (uint32_t i = 0; i < triCount; ++i) {
        STLTri t{};
        in.read(reinterpret_cast<char*>(&t), sizeof(STLTri));
        if (!in) {
            throw std::runtime_error("Unexpected EOF in STL: " + path);
        }

        out.positions.push_back(toVec3(t.v0));
        out.positions.push_back(toVec3(t.v1));
        out.positions.push_back(toVec3(t.v2));
    }

    return out;
}
