#include "Primitives.h"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

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

            glm::vec3 p0(R * sin(phi0) * cos(th0), R * sin(phi0) * sin(th0), R * cos(phi0));
            glm::vec3 p1(R * sin(phi0) * cos(th1), R * sin(phi0) * sin(th1), R * cos(phi0));
            glm::vec3 p2(R * sin(phi1) * cos(th0), R * sin(phi1) * sin(th0), R * cos(phi1));
            glm::vec3 p3(R * sin(phi1) * cos(th1), R * sin(phi1) * sin(th1), R * cos(phi1));

            push(p0.x, p0.y, p0.z); push(p2.x, p2.y, p2.z); push(p1.x, p1.y, p1.z);
            push(p1.x, p1.y, p1.z); push(p2.x, p2.y, p2.z); push(p3.x, p3.y, p3.z);
        }
    }
    return v;
}
