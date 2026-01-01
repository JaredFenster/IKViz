#pragma once
#include <string>
#include <vector>
#include <glm/glm.hpp>

struct STLMeshData {
    std::vector<glm::vec3> positions; // triangles, 3 per face
};

namespace STLLoader {
    STLMeshData LoadBinary(const std::string& path);
}
