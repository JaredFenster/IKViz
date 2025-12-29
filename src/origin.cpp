#include "Origin.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <stdexcept>
#include <cmath>

Origin::Origin(const glm::vec3& position,
               int xDir,
               int zDir,
               float axisLength,
               int rotateAxis,
               float angleDeg,
               float axisRadius,
               int cylinderSlices)
    : pos(position),
      length(axisLength),
      rotateAxisInt_(rotateAxis),
      angleDeg_(0.0f),
      radius_(axisRadius),
      slices_((cylinderSlices < 6) ? 6 : cylinderSlices)
{
    xAxis = dirFromInt(xDir);
    zAxis = dirFromInt(zDir);

    if (glm::length(glm::cross(xAxis, zAxis)) < 0.5f) {
        throw std::runtime_error("xDir and zDir cannot be parallel");
    }

    // right-handed basis
    yAxis = glm::normalize(glm::cross(zAxis, xAxis));
    orthonormalizeBasis_();

    // Build cylinder mesh + GL
    rebuildUnitCylinderMesh_();
    setupBuffers();

    // Apply initial rotation into the stored axes
    if (angleDeg != 0.0f) {
        rotateAboutAxisInt(rotateAxisInt_, angleDeg);
        angleDeg_ = angleDeg;
    }

    // Cache initial state for reset
    initPos_      = pos;
    initXAxis_    = xAxis;
    initYAxis_    = yAxis;
    initZAxis_    = zAxis;
    initAngleDeg_ = angleDeg_;
}

glm::vec3 Origin::dirFromInt(int d) const {
    int s = (d < 0) ? -1 : 1;
    int a = (d < 0) ? -d : d;

    switch (a) {
        case 1: return glm::vec3((float)s, 0.0f, 0.0f);
        case 2: return glm::vec3(0.0f, (float)s, 0.0f);
        case 3: return glm::vec3(0.0f, 0.0f, (float)s);
        default:
            throw std::runtime_error("Invalid axis int (use ±1, ±2, ±3)");
    }
}

glm::vec3 Origin::axisIntToWorld_(int axisInt) const {
    int s = (axisInt < 0) ? -1 : 1;
    int a = (axisInt < 0) ? -axisInt : axisInt;

    switch (a) {
        case 1: return (float)s * xAxis;
        case 2: return (float)s * yAxis;
        case 3: return (float)s * zAxis;
        default:
            throw std::runtime_error("Invalid axis int (use ±1, ±2, ±3)");
    }
}

glm::vec3 Origin::rotateVecAroundWorldAxis_(const glm::vec3& v,
                                           const glm::vec3& axisWorld,
                                           float angleDeg) const
{
    glm::vec3 ax = glm::normalize(axisWorld);
    float rad = glm::radians(angleDeg);
    glm::mat4 R = glm::rotate(glm::mat4(1.0f), rad, ax);
    return glm::vec3(R * glm::vec4(v, 0.0f));
}

void Origin::orthonormalizeBasis_() {
    xAxis = glm::normalize(xAxis);

    zAxis = zAxis - glm::dot(zAxis, xAxis) * xAxis;
    zAxis = glm::normalize(zAxis);

    yAxis = glm::normalize(glm::cross(zAxis, xAxis));
    zAxis = glm::normalize(glm::cross(xAxis, yAxis));
}

void Origin::rotateAboutAxisInt(int axisInt, float deltaDeg) {
    glm::vec3 axisWorld = axisIntToWorld_(axisInt);

    xAxis = rotateVecAroundWorldAxis_(xAxis, axisWorld, deltaDeg);
    yAxis = rotateVecAroundWorldAxis_(yAxis, axisWorld, deltaDeg);
    zAxis = rotateVecAroundWorldAxis_(zAxis, axisWorld, deltaDeg);

    orthonormalizeBasis_();
}

void Origin::rotateAboutOtherOriginAxis(const Origin& other,
                                       int otherAxisInt,
                                       float deltaDeg,
                                       bool rotatePosition)
{
    glm::vec3 axisWorld = other.axisIntToWorld_(otherAxisInt);

    xAxis = rotateVecAroundWorldAxis_(xAxis, axisWorld, deltaDeg);
    yAxis = rotateVecAroundWorldAxis_(yAxis, axisWorld, deltaDeg);
    zAxis = rotateVecAroundWorldAxis_(zAxis, axisWorld, deltaDeg);
    orthonormalizeBasis_();

    if (rotatePosition) {
        glm::vec3 rel = pos - other.pos;
        rel = rotateVecAroundWorldAxis_(rel, axisWorld, deltaDeg);
        pos = other.pos + rel;
    }
}

void Origin::rotate(float deltaDeg) {
    rotateAboutAxisInt(rotateAxisInt_, deltaDeg);
    angleDeg_ += deltaDeg;
}

void Origin::setAngleDeg(float deg) {
    float delta = deg - angleDeg_;
    rotate(delta);
}

void Origin::reset() {
    pos      = initPos_;
    xAxis    = initXAxis_;
    yAxis    = initYAxis_;
    zAxis    = initZAxis_;
    angleDeg_ = initAngleDeg_;
}

// ---------------------- Cylinder mesh + GL ----------------------

static inline void pushPos(std::vector<float>& out, const glm::vec3& p) {
    out.push_back(p.x); out.push_back(p.y); out.push_back(p.z);
}

void Origin::rebuildUnitCylinderMesh_() {
    // Unit cylinder along +Z, z in [0,1], radius = 1
    // Vertex format: position ONLY (xyz). We will use constant vertex attribute for color.
    std::vector<float> posOnly;
    posOnly.reserve(slices_ * 12 * 3);

    const float twoPi = 6.28318530718f;

    auto ring = [&](int i, float z) -> glm::vec3 {
        float t = twoPi * (float)i / (float)slices_;
        return glm::vec3(std::cos(t), std::sin(t), z);
    };

    // side surface
    for (int i = 0; i < slices_; i++) {
        int j = (i + 1) % slices_;

        glm::vec3 a0 = ring(i, 0.0f);
        glm::vec3 a1 = ring(j, 0.0f);
        glm::vec3 b0 = ring(i, 1.0f);
        glm::vec3 b1 = ring(j, 1.0f);

        // tri 1: a0, b0, b1
        pushPos(posOnly, a0);
        pushPos(posOnly, b0);
        pushPos(posOnly, b1);

        // tri 2: a0, b1, a1
        pushPos(posOnly, a0);
        pushPos(posOnly, b1);
        pushPos(posOnly, a1);
    }

    if (capEnds_) {
        // cap at z=0
        for (int i = 0; i < slices_; i++) {
            int j = (i + 1) % slices_;
            pushPos(posOnly, glm::vec3(0,0,0));
            pushPos(posOnly, ring(j, 0.0f));
            pushPos(posOnly, ring(i, 0.0f));
        }

        // cap at z=1
        for (int i = 0; i < slices_; i++) {
            int j = (i + 1) % slices_;
            pushPos(posOnly, glm::vec3(0,0,1));
            pushPos(posOnly, ring(i, 1.0f));
            pushPos(posOnly, ring(j, 1.0f));
        }
    }

    // Upload will happen in setupBuffers(); store temp in a static-like way by rebuilding VBO each time
    // We'll stash into VBO right away by calling setupBuffers() after this.
    if (VBO) {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER,
                     (GLsizeiptr)(posOnly.size() * sizeof(float)),
                     posOnly.data(),
                     GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        vertexCount_ = (GLsizei)(posOnly.size() / 3);
    } else {
        // store count; actual upload after VAO/VBO created
        vertexCount_ = (GLsizei)(posOnly.size() / 3);
        // we need to keep the data around until setupBuffers; easiest: create buffers first in setupBuffers
        // so we will just recreate fully in setupBuffers() when VBO==0
    }

    // If VBO doesn't exist yet, setupBuffers() will rebuild again (simple + safe).
}

void Origin::setupBuffers() {
    // Create buffers if missing
    if (VAO == 0) glGenVertexArrays(1, &VAO);
    if (VBO == 0) glGenBuffers(1, &VBO);

    // Rebuild the unit cylinder data here (so first-time works cleanly)
    std::vector<float> posOnly;
    posOnly.reserve(slices_ * 12 * 3);

    const float twoPi = 6.28318530718f;

    auto ring = [&](int i, float z) -> glm::vec3 {
        float t = twoPi * (float)i / (float)slices_;
        return glm::vec3(std::cos(t), std::sin(t), z);
    };

    for (int i = 0; i < slices_; i++) {
        int j = (i + 1) % slices_;

        glm::vec3 a0 = ring(i, 0.0f);
        glm::vec3 a1 = ring(j, 0.0f);
        glm::vec3 b0 = ring(i, 1.0f);
        glm::vec3 b1 = ring(j, 1.0f);

        pushPos(posOnly, a0);
        pushPos(posOnly, b0);
        pushPos(posOnly, b1);

        pushPos(posOnly, a0);
        pushPos(posOnly, b1);
        pushPos(posOnly, a1);
    }

    if (capEnds_) {
        for (int i = 0; i < slices_; i++) {
            int j = (i + 1) % slices_;
            pushPos(posOnly, glm::vec3(0,0,0));
            pushPos(posOnly, ring(j, 0.0f));
            pushPos(posOnly, ring(i, 0.0f));
        }
        for (int i = 0; i < slices_; i++) {
            int j = (i + 1) % slices_;
            pushPos(posOnly, glm::vec3(0,0,1));
            pushPos(posOnly, ring(i, 1.0f));
            pushPos(posOnly, ring(j, 1.0f));
        }
    }

    vertexCount_ = (GLsizei)(posOnly.size() / 3);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(posOnly.size() * sizeof(float)),
                 posOnly.data(),
                 GL_STATIC_DRAW);

    // position at location 0
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // color at location 1 will be a CONSTANT attribute for this VAO
    glDisableVertexAttribArray(1); // IMPORTANT: constant color via glVertexAttrib3f()

    glBindVertexArray(0);
}

void Origin::setCylinderSlices(int s) {
    slices_ = (s < 6) ? 6 : s;
    // rebuild VBO contents
    setupBuffers();
}

// ---------------------- Drawing ----------------------

glm::mat4 Origin::basisFromAxisDir_(const glm::vec3& axisDirWorld) {
    glm::vec3 w = glm::normalize(axisDirWorld);
    glm::vec3 helper = (std::abs(w.z) < 0.9f) ? glm::vec3(0,0,1) : glm::vec3(0,1,0);
    glm::vec3 u = glm::normalize(glm::cross(helper, w));
    glm::vec3 v = glm::normalize(glm::cross(w, u));

    // columns: u v w
    glm::mat4 B(1.0f);
    B[0] = glm::vec4(u, 0.0f);
    B[1] = glm::vec4(v, 0.0f);
    B[2] = glm::vec4(w, 0.0f);
    return B;
}

void Origin::drawAxisCylinder_(GLuint program, GLint uModelLoc,
                              const glm::vec3& axisDirWorld,
                              const glm::vec3& color) const
{
    // unit cylinder is along +Z from 0..1.
    // model = T(pos) * R(z->axisDirWorld) * S(radius, radius, length)
    glm::mat4 T = glm::translate(glm::mat4(1.0f), pos);
    glm::mat4 R = basisFromAxisDir_(axisDirWorld);
    glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(radius_, radius_, length));

    glm::mat4 model = T * R * S;

    glUseProgram(program);
    glUniformMatrix4fv(uModelLoc, 1, GL_FALSE, glm::value_ptr(model));

    // constant vertex color (attribute location = 1)
    glBindVertexArray(VAO);
    glVertexAttrib3f(1, color.r, color.g, color.b);
    glDrawArrays(GL_TRIANGLES, 0, vertexCount_);
    glBindVertexArray(0);
}

void Origin::draw(GLuint program, GLint uModelLoc) const {
    // Draw 3 axis cylinders with standard colors
    drawAxisCylinder_(program, uModelLoc, xAxis, glm::vec3(1,0,0));
    drawAxisCylinder_(program, uModelLoc, yAxis, glm::vec3(0,1,0));
    drawAxisCylinder_(program, uModelLoc, zAxis, glm::vec3(0,0,1));
}

// ---------------------- Move + destroy ----------------------

Origin::Origin(Origin&& other) noexcept {
    *this = std::move(other);
}

Origin& Origin::operator=(Origin&& other) noexcept {
    if (this == &other) return *this;

    if (VBO) glDeleteBuffers(1, &VBO);
    if (VAO) glDeleteVertexArrays(1, &VAO);

    VAO = other.VAO; other.VAO = 0;
    VBO = other.VBO; other.VBO = 0;
    vertexCount_ = other.vertexCount_; other.vertexCount_ = 0;

    pos = other.pos;
    xAxis = other.xAxis; yAxis = other.yAxis; zAxis = other.zAxis;
    length = other.length;

    rotateAxisInt_ = other.rotateAxisInt_;
    angleDeg_ = other.angleDeg_;

    radius_ = other.radius_;
    slices_ = other.slices_;
    capEnds_ = other.capEnds_;

    initPos_ = other.initPos_;
    initXAxis_ = other.initXAxis_;
    initYAxis_ = other.initYAxis_;
    initZAxis_ = other.initZAxis_;
    initAngleDeg_ = other.initAngleDeg_;

    return *this;
}

Origin::~Origin() {
    if (VBO) glDeleteBuffers(1, &VBO);
    if (VAO) glDeleteVertexArrays(1, &VAO);
}
