#pragma once
#include "origin.h"
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <vector>

class linkJoint {
public:
    linkJoint(Origin& Base);
    void addJoint(Origin& NewJoint);

    void rotateJoint(int num, float angleDeg);
    void reset();

    void link(GLuint program, GLint uModelLoc) const;

    void addVerts();
    void setupBuffers();

    // Cylinder controls
    void setLinkRadius(float r) { linkRadius_ = r; }
    void setLinkSlices(int s)   { linkSlices_ = (s < 6) ? 6 : s; } // minimum sane
    friend class IK;
    std::vector<float> verts;
    std::vector<Origin*> robot;

private:
    // Cylinder params
    float linkRadius_ = 0.03f;
    int   linkSlices_ = 16;
    bool  capEnds_    = true;


    GLsizei vertexCount = 0;

    GLuint lineVAO = 0;
    GLuint lineVBO = 0;

    // helpers
    static void pushVertex(std::vector<float>& verts, const glm::vec3& p, const glm::vec3& c);
    void addCylinderSegment_(const glm::vec3& a, const glm::vec3& b);
};
