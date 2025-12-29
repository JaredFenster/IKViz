#pragma once
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <glm/gtc/quaternion.hpp>


class Origin {
public:
        // Limits
    void setLimitsDeg(float minDeg, float maxDeg) {
        minDeg_ = minDeg;
        maxDeg_ = maxDeg;
        limitsEnabled_ = true;
        // optional: clamp current angle to be safe
        setAngleDeg(angleDeg_);
    }
    
    void clearLimits() { limitsEnabled_ = false; }
    
    bool  hasLimits() const { return limitsEnabled_; }
    float getMinDeg() const { return minDeg_; }
    float getMaxDeg() const { return maxDeg_; }
    
    // For IK convenience (clamp without changing enable state)
    float clampToLimits(float deg) const {
        if (!limitsEnabled_) return deg;
        if (deg < minDeg_) return minDeg_;
        if (deg > maxDeg_) return maxDeg_;
        return deg;
    }




    Origin(const glm::vec3& position,
       int xDir,
       int zDir,
       float axisLength,
       int rotateAxis,
       float angleDeg,
       float axisRadius = 0.02f,
       int cylinderSlices = 16,
       float minDeg = -180.0f,
       float maxDeg =  180.0f,
       bool enableLimits = false);




    Origin(const Origin&) = delete;
    Origin& operator=(const Origin&) = delete;

    Origin(Origin&& other) noexcept;
    Origin& operator=(Origin&& other) noexcept;

    ~Origin();

    void draw(GLuint program, GLint uModelLoc) const;

    // Self rotation about rotateAxisInt_
    void rotate(float deltaDeg);
    void setAngleDeg(float deg);

    // Rotate about one of THIS origin's LOCAL axes (±1/±2/±3)
    void rotateAboutAxisInt(int axisInt, float deltaDeg);

    // Rotate about ANOTHER origin's LOCAL axis (±1/±2/±3). If rotatePosition=true, orbit pos around other.pos.
    void rotateAboutOtherOriginAxis(const Origin& other,
                                   int otherAxisInt,
                                   float deltaDeg,
                                   bool rotatePosition = true);

    // Reset to initial pose
    void reset();

    // Visual controls
    void setAxisLength(float L) { length = L; }
    void setAxisRadius(float r) { radius_ = r; }           // no rebuild needed
    void setCylinderSlices(int s);                         // rebuilds mesh

    float getX() const { return pos.x; }
    float getY() const { return pos.y; }
    float getZ() const { return pos.z; }

    // If you already have this API used elsewhere:
    int getAxis() const { return rotateAxisInt_; }
    // ---- IK accessors ----
    const glm::vec3& getPos() const { return pos; }
        
    const glm::vec3& getXAxis() const { return xAxis; }
    const glm::vec3& getYAxis() const { return yAxis; }
    const glm::vec3& getZAxis() const { return zAxis; }
        
    int   getRotateAxisInt() const { return rotateAxisInt_; }
    float getAngleDeg() const { return angleDeg_; }
        
    // Returns +/− xAxis/yAxis/zAxis depending on axisInt
    glm::vec3 axisIntToWorldPublic(int axisInt) const { return axisIntToWorld_(axisInt); }
        
    // Convenience: the axis this joint rotates about (world space)
    glm::vec3 getJointAxisWorld() const { return axisIntToWorld_(rotateAxisInt_); }

    glm::mat3 getRotationMat3() const {
        // columns are the basis vectors in world space
        return glm::mat3(xAxis, yAxis, zAxis);
    }

    glm::quat getRotationQuat() const {
        return glm::quat_cast(getRotationMat3());
    }



private:
    glm::vec3 pos{}, xAxis{}, yAxis{}, zAxis{};
    float length = 0.2f;

    float minDeg_ = -180.0f;
    float maxDeg_ =  180.0f;
    bool  limitsEnabled_ = false;


    int rotateAxisInt_ = 3;
    float angleDeg_ = 0.0f;

    // cylinder look
    float radius_ = 0.02f;
    int slices_ = 16;
    bool capEnds_ = false;

    // cached initial pose for reset
    glm::vec3 initPos_{};
    glm::vec3 initXAxis_{}, initYAxis_{}, initZAxis_{};
    float initAngleDeg_ = 0.0f;

    GLuint VAO = 0;
    GLuint VBO = 0;
    GLsizei vertexCount_ = 0;

    // mesh build + GL buffers
    void setupBuffers();
    void rebuildUnitCylinderMesh_(); // builds unit cylinder along +Z, z in [0,1], radius=1

    // axis helpers
    glm::vec3 dirFromInt(int d) const;
    glm::vec3 axisIntToWorld_(int axisInt) const;

    glm::vec3 rotateVecAroundWorldAxis_(const glm::vec3& v,
                                        const glm::vec3& axisWorld,
                                        float angleDeg) const;

    void orthonormalizeBasis_();

    // Build a rotation matrix whose columns are an orthonormal basis (u,v,w)
    // where w is the desired axis direction in world space.
    static glm::mat4 basisFromAxisDir_(const glm::vec3& axisDirWorld);

    // Draw a cylinder aligned with axisDirWorld, colored rgb
    void drawAxisCylinder_(GLuint program, GLint uModelLoc,
                           const glm::vec3& axisDirWorld,
                           const glm::vec3& color) const;
};
