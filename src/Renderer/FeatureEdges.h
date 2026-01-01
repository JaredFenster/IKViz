#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "STLLoader.h"

// Returns interleaved [x y z r g b] for GL_LINES
std::vector<float> BuildFeatureEdgeLines(
    const STLMeshData& stl,
    float weldEps,          // vertex weld tolerance in STL units
    float creaseAngleDeg,   // draw edges sharper than this
    const glm::vec3& color  // line color
);
