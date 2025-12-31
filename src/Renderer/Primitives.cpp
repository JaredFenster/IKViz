#include "Primitives.h"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

std::vector<float> makeGridVerts(int gridHalf) {
    std::vector<float> v;
    v.reserve(((2 * gridHalf + 1) * 2 * 2 * 6));

    auto push = [&](float x, float y, float z, float r, float g, float b) {
        v.push_back(x); v.push_back(y); v.push_back(z);
        v.push_back(r); v.push_back(g); v.push_back(b);
    };

    // axis lines
    push((float)-gridHalf, 0, 0, 1, 0, 0); push((float)+gridHalf, 0, 0, 1, 0, 0);
    push(0, (float)-gridHalf, 0, 0, 1, 0); push(0, (float)+gridHalf, 0, 0, 1, 0);
    push(0, 0, 0, 0, 0, 1);              push(0, 0, 2, 0, 0, 1);

    for (int i = -gridHalf; i <= gridHalf; ++i) {
        if (i == 0) continue;
        float r = 0.25f, g = 0.25f, b = 0.25f;
        float x = (float)i, y = (float)i;

        push(x, (float)-gridHalf, 0, r, g, b);
        push(x, (float)+gridHalf, 0, r, g, b);

        push((float)-gridHalf, y, 0, r, g, b);
        push((float)+gridHalf, y, 0, r, g, b);
    }

    return v;
}

std::vector<float> makeSphereVerts(float R, int slices, int stacks) {
    std::vector<float> v;

    auto push = [&](float x, float y, float z) {
        v.push_back(x); v.push_back(y); v.push_back(z);
        v.push_back(1.0f); v.push_back(1.0f); v.push_back(0.0f);
    };

    for (int i = 0; i < stacks; ++i) {
        float phi0 = glm::pi<float>() * (float)i / stacks;
        float phi1 = glm::pi<float>() * (float)(i + 1) / stacks;

        for (int j = 0; j < slices; ++j) {
            float th0 = glm::two_pi<float>() * (float)j / slices;
            float th1 = glm::two_pi<float>() * (float)(j + 1) / slices;

            glm::vec3 p0(R * std::sin(phi0) * std::cos(th0), R * std::sin(phi0) * std::sin(th0), R * std::cos(phi0));
            glm::vec3 p1(R * std::sin(phi0) * std::cos(th1), R * std::sin(phi0) * std::sin(th1), R * std::cos(phi0));
            glm::vec3 p2(R * std::sin(phi1) * std::cos(th0), R * std::sin(phi1) * std::sin(th0), R * std::cos(phi1));
            glm::vec3 p3(R * std::sin(phi1) * std::cos(th1), R * std::sin(phi1) * std::sin(th1), R * std::cos(phi1));

            push(p0.x, p0.y, p0.z); push(p2.x, p2.y, p2.z); push(p1.x, p1.y, p1.z);
            push(p1.x, p1.y, p1.z); push(p2.x, p2.y, p2.z); push(p3.x, p3.y, p3.z);
        }
    }
    return v;
}

std::vector<float> makeCylinderVerts(float radius, float height, int segments) {
    std::vector<float> v;

    segments = std::max(segments, 3);

    auto push = [&](float x, float y, float z, float r, float g, float b) {
        v.push_back(x); v.push_back(y); v.push_back(z);
        v.push_back(r); v.push_back(g); v.push_back(b);
    };

    // White by default (easy to tint later if you switch gizmo shader to uniform color)
    const float cr = 1.0f, cg = 1.0f, cb = 1.0f;

    const float twoPi = glm::two_pi<float>();

    glm::vec3 c0(0.0f, 0.0f, 0.0f);
    glm::vec3 c1(0.0f, 0.0f, height);

    for (int i = 0; i < segments; ++i) {
        float a0 = twoPi * (float)i / (float)segments;
        float a1 = twoPi * (float)(i + 1) / (float)segments;

        float x0 = radius * std::cos(a0);
        float y0 = radius * std::sin(a0);
        float x1 = radius * std::cos(a1);
        float y1 = radius * std::sin(a1);

        glm::vec3 p00(x0, y0, 0.0f);
        glm::vec3 p01(x1, y1, 0.0f);
        glm::vec3 p10(x0, y0, height);
        glm::vec3 p11(x1, y1, height);

        // side wall
        push(p00.x, p00.y, p00.z, cr, cg, cb);
        push(p10.x, p10.y, p10.z, cr, cg, cb);
        push(p11.x, p11.y, p11.z, cr, cg, cb);

        push(p00.x, p00.y, p00.z, cr, cg, cb);
        push(p11.x, p11.y, p11.z, cr, cg, cb);
        push(p01.x, p01.y, p01.z, cr, cg, cb);

        // top cap (+Z)
        push(c1.x, c1.y, c1.z, cr, cg, cb);
        push(p11.x, p11.y, p11.z, cr, cg, cb);
        push(p10.x, p10.y, p10.z, cr, cg, cb);

        // bottom cap (-Z)
        push(c0.x, c0.y, c0.z, cr, cg, cb);
        push(p00.x, p00.y, p00.z, cr, cg, cb);
        push(p01.x, p01.y, p01.z, cr, cg, cb);
    }

    return v;
}
