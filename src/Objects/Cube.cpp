#include "Cube.h"

#include "../Renderer/Mesh.h"
#include "../Renderer/Shader.h"
#include "../InverseKinematics/IK.h"   // for URDFIK::ComputeFK / FKResult / ChainInfo

#include <vector>

static Mesh& SharedCubeMesh()
{
    // Unit cube centered at origin, side length 1.
    // 12 triangles = 36 vertices.
    // Vertex format: pos(3) + color(3)
    static Mesh cubeMesh;
    static bool inited = false;

    if (!inited)
    {
        std::vector<float> v;
        v.reserve(36 * 6);

        auto push = [&](float x, float y, float z, float r, float g, float b)
        {
            v.push_back(x); v.push_back(y); v.push_back(z);
            v.push_back(r); v.push_back(g); v.push_back(b);
        };

        // corners (±0.5)
        const float p = 0.5f;
        const float n = -0.5f;

        // color (white — you can tint using shader uniform if you want)
        const float cr = 1.0f, cg = 1.0f, cb = 1.0f;

        // helper: make a face from 4 corners (two triangles)
        auto quad = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d)
        {
            // (a,b,c) and (a,c,d)
            push(a.x,a.y,a.z, cr,cg,cb);
            push(b.x,b.y,b.z, cr,cg,cb);
            push(c.x,c.y,c.z, cr,cg,cb);

            push(a.x,a.y,a.z, cr,cg,cb);
            push(c.x,c.y,c.z, cr,cg,cb);
            push(d.x,d.y,d.z, cr,cg,cb);
        };

        // Define faces (CCW doesn’t matter for you unless you enable culling)
        // +X
        quad({p,n,n}, {p,p,n}, {p,p,p}, {p,n,p});
        // -X
        quad({n,n,p}, {n,p,p}, {n,p,n}, {n,n,n});
        // +Y
        quad({n,p,n}, {n,p,p}, {p,p,p}, {p,p,n});
        // -Y
        quad({n,n,p}, {n,n,n}, {p,n,n}, {p,n,p});
        // +Z
        quad({n,n,p}, {p,n,p}, {p,p,p}, {n,p,p});
        // -Z
        quad({p,n,n}, {n,n,n}, {n,p,n}, {p,p,n});

        cubeMesh = Mesh::fromVertexColorTriangles(v);
        inited = true;
    }

    return cubeMesh;
}

Cube::Cube(const glm::vec3& position_,
           const glm::quat& rotation_,
           const glm::vec3& scale_)
    : position(position_), rotation(rotation_), scale(scale_)
{
}

void Cube::SetParent(RobotScene* parent, const URDFIK::ChainInfo* chain)
{
    parent_ = parent;
    chain_ = chain;
}

void Cube::UpdateFromParentEE()
{
    if (!grabbed) return;
    if (!parent_ || !chain_) return;

    // Compute FK for parent's end effector pose
    std::vector<glm::mat4> jf;
    URDFIK::FKResult fk = URDFIK::ComputeFK(parent_->Robot(), *chain_, jf);

    position = fk.pos;
    rotation = fk.rot;
}

void Cube::Draw(Shader& shader) const
{
    glm::mat4 M(1.0f);
    M = glm::translate(M, position);
    M *= glm::mat4_cast(glm::normalize(rotation));
    M = glm::scale(M, scale);

    shader.setMat4("uModel", M);

    // If you want cube uniform color + alpha, do it here.
    // Otherwise leave uUseUniformColor=false and let vertex colors show.
    // (Your shader defaults in App set uUseUniformColor=false)
    SharedCubeMesh().drawTriangles();
}

void Cube::SetGrabbed(bool g)
{
    std::vector<glm::mat4> jf;
    URDFIK::FKResult fk = URDFIK::ComputeFK(parent_->Robot(), *chain_, jf);

    //if ee vector == grab vector
}