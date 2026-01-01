#include "FeatureEdges.h"
#include <unordered_map>
#include <cmath>
#include <cstdint>
#include <algorithm>

static glm::vec3 triNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 n = glm::cross(b - a, c - a);
    float len = glm::length(n);
    if (len < 1e-12f) return glm::vec3(0,0,0);
    return n / len;
}

// Quantize position to an integer grid so identical-ish vertices weld
struct QVec3 {
    int32_t x, y, z;
    bool operator==(const QVec3& o) const { return x==o.x && y==o.y && z==o.z; }
};

struct QHash {
    size_t operator()(const QVec3& v) const noexcept {
        // simple mix
        size_t h = (size_t)v.x * 73856093u;
        h ^= (size_t)v.y * 19349663u;
        h ^= (size_t)v.z * 83492791u;
        return h;
    }
};

static QVec3 quantize(const glm::vec3& p, float eps) {
    auto q = [&](float f) -> int32_t { return (int32_t)std::llround((double)f / (double)eps); };
    return { q(p.x), q(p.y), q(p.z) };
}

struct EdgeKey {
    uint32_t a, b; // sorted
    bool operator==(const EdgeKey& o) const { return a==o.a && b==o.b; }
};

struct EdgeHash {
    size_t operator()(const EdgeKey& e) const noexcept {
        return ((size_t)e.a << 32) ^ (size_t)e.b;
    }
};

struct EdgeInfo {
    uint32_t v0, v1;
    glm::vec3 n0{0}, n1{0};
    int faces = 0;
};

std::vector<float> BuildFeatureEdgeLines(
    const STLMeshData& stl,
    float weldEps,
    float creaseAngleDeg,
    const glm::vec3& color
) {
    // 1) Weld vertices by quantization -> unique vertex list
    std::unordered_map<QVec3, uint32_t, QHash> qToIndex;
    std::vector<glm::vec3> verts;
    verts.reserve(stl.positions.size());

    auto getIndex = [&](const glm::vec3& p) -> uint32_t {
        QVec3 q = quantize(p, weldEps);
        auto it = qToIndex.find(q);
        if (it != qToIndex.end()) return it->second;
        uint32_t idx = (uint32_t)verts.size();
        qToIndex.emplace(q, idx);
        verts.push_back(p);
        return idx;
    };

    // 2) Build edge map with adjacent face normals
    std::unordered_map<EdgeKey, EdgeInfo, EdgeHash> edges;
    const auto& P = stl.positions;

    auto addEdge = [&](uint32_t i0, uint32_t i1, const glm::vec3& n) {
        uint32_t a = std::min(i0, i1);
        uint32_t b = std::max(i0, i1);
        EdgeKey key{a,b};

        auto& e = edges[key];
        if (e.faces == 0) {
            e.v0 = a; e.v1 = b;
            e.n0 = n;
            e.faces = 1;
        } else if (e.faces == 1) {
            e.n1 = n;
            e.faces = 2;
        } else {
            // ignore non-manifold extra faces
        }
    };

    for (size_t t = 0; t + 2 < P.size(); t += 3) {
        uint32_t i0 = getIndex(P[t+0]);
        uint32_t i1 = getIndex(P[t+1]);
        uint32_t i2 = getIndex(P[t+2]);

        glm::vec3 n = triNormal(verts[i0], verts[i1], verts[i2]);
        if (glm::length(n) < 0.5f) continue;

        addEdge(i0, i1, n);
        addEdge(i1, i2, n);
        addEdge(i2, i0, n);
    }

    // 3) Emit feature edges only
    const float creaseRad = creaseAngleDeg * 3.1415926535f / 180.0f;
    const float cosThresh = std::cos(creaseRad);

    std::vector<float> out; // [x y z r g b] * 2 per line
    out.reserve(edges.size() * 12);

    auto emit = [&](const glm::vec3& a, const glm::vec3& b) {
        out.insert(out.end(), {a.x,a.y,a.z, color.r,color.g,color.b});
        out.insert(out.end(), {b.x,b.y,b.z, color.r,color.g,color.b});
    };

    for (const auto& kv : edges) {
        const EdgeInfo& e = kv.second;
        bool isBoundary = (e.faces == 1);

        bool isCrease = false;
        if (e.faces == 2) {
            float d = glm::dot(e.n0, e.n1);
            // If normals differ a lot => crease
            isCrease = (d < cosThresh);
        }

        if (isBoundary || isCrease) {
            emit(verts[e.v0], verts[e.v1]);
        }
    }

    return out;
}
