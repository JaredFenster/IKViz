#include "linkJoint.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <stdexcept>
#include <cmath>

void linkJoint::pushVertex(std::vector<float>& verts,
                           const glm::vec3& p,
                           const glm::vec3& c)
{
    verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
    verts.push_back(c.r); verts.push_back(c.g); verts.push_back(c.b);
}

linkJoint::linkJoint(Origin& base) {
    robot.clear();
    verts.clear();
    robot.push_back(&base);
    addVerts();
    setupBuffers();
}

void linkJoint::addJoint(Origin& newJoint) {
    robot.push_back(&newJoint);
    addVerts();
    setupBuffers();
}

void linkJoint::setupBuffers() {
    if (lineVAO == 0) glGenVertexArrays(1, &lineVAO);
    if (lineVBO == 0) glGenBuffers(1, &lineVBO);

    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);

    glBufferData(GL_ARRAY_BUFFER,
                 verts.size() * sizeof(float),
                 verts.data(),
                 GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    vertexCount = (GLsizei)(verts.size() / 6);
}

// Build one cylinder between points a->b (CPU-generated triangles)
void linkJoint::addCylinderSegment_(const glm::vec3& a, const glm::vec3& b) {
    const glm::vec3 color(0.831f, 0.651f, 0.0f);

    glm::vec3 ab = b - a;
    float len = glm::length(ab);
    if (len < 1e-6f) return;

    glm::vec3 axis = ab / len;

    // Build an orthonormal basis (u, v, axis)
    // Pick a helper not parallel to axis
    glm::vec3 helper = (std::abs(axis.z) < 0.9f) ? glm::vec3(0, 0, 1) : glm::vec3(0, 1, 0);
    glm::vec3 u = glm::normalize(glm::cross(axis, helper));
    glm::vec3 v = glm::normalize(glm::cross(axis, u));

    const float r = linkRadius_;
    const int N = linkSlices_;
    const float twoPi = 6.28318530718f;

    auto ringPointA = [&](int i) {
        float t = twoPi * (float)i / (float)N;
        return a + r * (std::cos(t) * u + std::sin(t) * v);
    };
    auto ringPointB = [&](int i) {
        float t = twoPi * (float)i / (float)N;
        return b + r * (std::cos(t) * u + std::sin(t) * v);
    };

    // Side surface: N quads => 2 triangles each
    for (int i = 0; i < N; i++) {
        int j = (i + 1) % N;

        glm::vec3 a0 = ringPointA(i);
        glm::vec3 a1 = ringPointA(j);
        glm::vec3 b0 = ringPointB(i);
        glm::vec3 b1 = ringPointB(j);

        // tri 1: a0, b0, b1
        pushVertex(verts, a0, color);
        pushVertex(verts, b0, color);
        pushVertex(verts, b1, color);

        // tri 2: a0, b1, a1
        pushVertex(verts, a0, color);
        pushVertex(verts, b1, color);
        pushVertex(verts, a1, color);
    }

    // Optional caps (triangle fan). You can turn off by setting capEnds_=false in header.
    if (capEnds_) {
        // Cap at A: center a, winding chosen to be consistent-ish
        for (int i = 0; i < N; i++) {
            int j = (i + 1) % N;
            glm::vec3 p0 = a;
            glm::vec3 p1 = ringPointA(j);
            glm::vec3 p2 = ringPointA(i);
            pushVertex(verts, p0, color);
            pushVertex(verts, p1, color);
            pushVertex(verts, p2, color);
        }

        // Cap at B
        for (int i = 0; i < N; i++) {
            int j = (i + 1) % N;
            glm::vec3 p0 = b;
            glm::vec3 p1 = ringPointB(i);
            glm::vec3 p2 = ringPointB(j);
            pushVertex(verts, p0, color);
            pushVertex(verts, p1, color);
            pushVertex(verts, p2, color);
        }
    }
}

void linkJoint::addVerts() {
    verts.clear();
    if (robot.size() < 2) return;

    const int segments = (int)robot.size() - 1;

    // sides: segments * slices * 2 triangles * 3 verts = segments*slices*6 verts
    // caps:  segments * 2 ends * slices * 1 triangle * 3 verts = segments*slices*6 verts
    // total verts ~ segments*slices*12, each vert 6 floats
    verts.reserve(segments * linkSlices_ * 12 * 6);

    for (int i = 0; i + 1 < (int)robot.size(); i++) {
        glm::vec3 a(robot[i]->getX(),   robot[i]->getY(),   robot[i]->getZ());
        glm::vec3 b(robot[i+1]->getX(), robot[i+1]->getY(), robot[i+1]->getZ());
        addCylinderSegment_(a, b);
    }

    vertexCount = (GLsizei)(verts.size() / 6);
}

void linkJoint::link(GLuint program, GLint uModelLoc) const {
    glm::mat4 model(1.0f);
    glUseProgram(program);
    glUniformMatrix4fv(uModelLoc, 1, GL_FALSE, glm::value_ptr(model));

    glBindVertexArray(lineVAO);
    glDrawArrays(GL_TRIANGLES, 0, vertexCount);
    glBindVertexArray(0);
}

void linkJoint::rotateJoint(int num, float angleDeg) {
    robot.at(num)->rotate(angleDeg);

    for (int i = num + 1; i < (int)robot.size(); i++) {
        robot.at(i)->rotateAboutOtherOriginAxis(*robot.at(num),
                                               robot.at(num)->getAxis(),
                                               angleDeg,
                                               true);
    }

    addVerts();
    setupBuffers();
}

void linkJoint::reset() {
    for (int i = 0; i < (int)robot.size(); i++) {
        robot.at(i)->reset();
    }
    addVerts();
    setupBuffers();
}
