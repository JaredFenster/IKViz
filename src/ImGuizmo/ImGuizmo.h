#pragma once
/*
  ImGuizmoLite.h (lightweight, header-only transform gizmo)

  - Translation arrows (X/Y/Z)
  - Rotation rings (X/Y/Z)
  - Ray pick + drag updates a target pose
  - Drawing uses a user callback: drawLine(p0, p1, rgb)

  ORTHO SIZE FIX:
  - Gizmo feature sizes are derived from the ORTHO projection scale, not camera->EE distance.
  - This keeps gizmo features the same on-screen size while orbiting in orthographic view.

  Dependencies: GLM
*/

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <optional>

namespace ImGuizmoLite {

struct Ray {
    glm::vec3 o;
    glm::vec3 d; // normalized
};

enum class Handle {
    None,
    MoveX, MoveY, MoveZ,
    RotX,  RotY,  RotZ
};

struct Pose {
    glm::vec3 pos{0.0f};
    glm::quat rot{1,0,0,0}; // (w,x,y,z)
};

struct Settings {
    // Gizmo sizes in *ortho-radius units* (scaled by orthoRadius = 1/proj[1][1])
    float baseAxisLen     = 0.25f;
    float baseRingRadius  = 0.18f;

    // Pick thresholds in ortho-radius units
    float basePickAxisThickness = 0.03f;
    float basePickRingThickness = 0.03f;

    // drawing resolution
    int ringSegments = 64;

    // colors
    glm::vec3 colX{1,0,0};
    glm::vec3 colY{0,1,0};
    glm::vec3 colZ{0,0,1};
    glm::vec3 colHover{1,1,0};
    glm::vec3 colActive{1,0.6f,0};

    // If you want rings the same color as axis:
    bool ringUsesAxisColor = true;
};

class Gizmo {
public:
    Settings s;

    Handle hovered = Handle::None;
    Handle active  = Handle::None;

    bool capturingMouse = false;

    Pose target;

    Gizmo() = default;

    void setTarget(const Pose& p) { target = p; }

    // NOTE: camPos is not needed for sizing anymore, but kept out of the signature
    // to avoid changing your app code too much. Sizing is derived from proj.
    void update(
        const Pose& ee,
        const glm::mat4& view,
        const glm::mat4& proj,
        const glm::vec3& /*camPos*/,
        const glm::vec3& camForward,
        float mouseX, float mouseY,
        float viewportW, float viewportH,
        bool lmbDown, bool lmbPressed, bool lmbReleased
    ){
        // Ortho-consistent sizing: orthoRadius is the half-height in world units
        // for a standard glm::ortho(-r*aspect, r*aspect, -r, r, ...)
        float orthoRadius = orthoRadiusFromProj(proj);

        float axisLen    = s.baseAxisLen * orthoRadius;
        float ringR      = s.baseRingRadius * orthoRadius;
        float pickAxisTh = s.basePickAxisThickness * orthoRadius;
        float pickRingTh = s.basePickRingThickness * orthoRadius;

        Ray ray = makeMouseRayWorld(mouseX, mouseY, viewportW, viewportH, view, proj);

        capturingMouse = (active != Handle::None);

        if (active == Handle::None) {
            hovered = hitTest(ray, ee, axisLen, ringR, pickAxisTh, pickRingTh);
        }

        if (lmbPressed && hovered != Handle::None) {
            beginDrag(hovered, ray, ee, target, camForward, ringR);
            capturingMouse = true;
        }

        if (active != Handle::None && lmbDown) {
            updateDrag(ray, ee, target, ringR);
            capturingMouse = true;
        }

        if (lmbReleased) {
            endDrag();
            capturingMouse = false;
        }
    }

    // Drawing: pass proj so sizing matches ortho exactly.
    template <typename DrawLineFn>
    void draw(const Pose& ee, const glm::mat4& proj, DrawLineFn&& drawLine) const {
        float orthoRadius = orthoRadiusFromProj(proj);

        float axisLen = s.baseAxisLen * orthoRadius;
        float ringR   = s.baseRingRadius * orthoRadius;

        glm::vec3 X = ee.rot * glm::vec3(1,0,0);
        glm::vec3 Y = ee.rot * glm::vec3(0,1,0);
        glm::vec3 Z = ee.rot * glm::vec3(0,0,1);

        auto colFor = [&](Handle h, const glm::vec3& axisCol)->glm::vec3{
            if (active == h)  return s.colActive;
            if (hovered == h) return s.colHover;
            return axisCol;
        };

        // arrows (simple lines; add arrowheads later if you want)
        drawLine(ee.pos, ee.pos + X * axisLen, colFor(Handle::MoveX, s.colX));
        drawLine(ee.pos, ee.pos + Y * axisLen, colFor(Handle::MoveY, s.colY));
        drawLine(ee.pos, ee.pos + Z * axisLen, colFor(Handle::MoveZ, s.colZ));

        // rings
        drawRing(ee.pos, X, ringR, colFor(Handle::RotX, s.ringUsesAxisColor ? s.colX : glm::vec3(1)), drawLine);
        drawRing(ee.pos, Y, ringR, colFor(Handle::RotY, s.ringUsesAxisColor ? s.colY : glm::vec3(1)), drawLine);
        drawRing(ee.pos, Z, ringR, colFor(Handle::RotZ, s.ringUsesAxisColor ? s.colZ : glm::vec3(1)), drawLine);
    }

private:
    // ----- helpers -----

    static float orthoRadiusFromProj(const glm::mat4& proj) {
        // For glm::ortho(..., bottom=-r, top=+r) -> proj[1][1] = 1/r
        float yy = proj[1][1];
        if (std::fabs(yy) < 1e-8f) return 1.0f;
        return 1.0f / yy;
    }

    static Ray makeMouseRayWorld(
        float mouseX, float mouseY,
        float viewportW, float viewportH,
        const glm::mat4& view,
        const glm::mat4& proj
    ){
        float x = (2.0f * mouseX) / viewportW - 1.0f;
        float y = 1.0f - (2.0f * mouseY) / viewportH;

        glm::mat4 invVP = glm::inverse(proj * view);

        glm::vec4 nearP = invVP * glm::vec4(x, y, -1.0f, 1.0f);
        glm::vec4 farP  = invVP * glm::vec4(x, y,  1.0f, 1.0f);
        nearP /= nearP.w;
        farP  /= farP.w;

        glm::vec3 o = glm::vec3(nearP);
        glm::vec3 d = glm::normalize(glm::vec3(farP - nearP));
        return {o, d};
    }

    static std::optional<float> rayPlaneT(const Ray& r, const glm::vec3& p0, const glm::vec3& n) {
        float denom = glm::dot(n, r.d);
        if (std::fabs(denom) < 1e-6f) return std::nullopt;
        float t = glm::dot(p0 - r.o, n) / denom;
        if (t < 0.0f) return std::nullopt;
        return t;
    }

    static std::optional<glm::vec3> rayIntersectPlanePoint(const Ray& r, const glm::vec3& p0, const glm::vec3& n) {
        auto t = rayPlaneT(r, p0, n);
        if (!t) return std::nullopt;
        return r.o + (*t) * r.d;
    }

    static float distanceRaySegment(const Ray& r, const glm::vec3& a, const glm::vec3& b) {
        glm::vec3 v  = b - a;
        glm::vec3 w0 = r.o - a;

        float A = glm::dot(r.d, r.d); // 1
        float B = glm::dot(r.d, v);
        float C = glm::dot(v, v);
        float D = glm::dot(r.d, w0);
        float E = glm::dot(v, w0);

        float denom = A*C - B*B;
        float sN, tN;

        if (std::fabs(denom) < 1e-6f) {
            sN = 0.0f;
            tN = (C > 1e-6f) ? (E / C) : 0.0f;
        } else {
            sN = (B*E - C*D) / denom;
            tN = (A*E - B*D) / denom;
        }

        if (sN < 0.0f) sN = 0.0f;
        tN = glm::clamp(tN, 0.0f, 1.0f);

        glm::vec3 pRay = r.o + sN * r.d;
        glm::vec3 pSeg = a + tN * v;
        return glm::length(pRay - pSeg);
    }

    static Handle hitTest(
        const Ray& ray,
        const Pose& ee,
        float axisLen,
        float ringR,
        float pickAxisTh,
        float pickRingTh
    ){
        glm::vec3 X = ee.rot * glm::vec3(1,0,0);
        glm::vec3 Y = ee.rot * glm::vec3(0,1,0);
        glm::vec3 Z = ee.rot * glm::vec3(0,0,1);

        Handle best = Handle::None;
        float bestVal = 1e9f;

        auto tryAxis = [&](Handle h, const glm::vec3& dir){
            float d = distanceRaySegment(ray, ee.pos, ee.pos + dir * axisLen);
            if (d < pickAxisTh && d < bestVal) { bestVal = d; best = h; }
        };

        auto tryRing = [&](Handle h, const glm::vec3& n){
            auto q = rayIntersectPlanePoint(ray, ee.pos, n);
            if (!q) return;
            float r = glm::length(*q - ee.pos);
            float err = std::fabs(r - ringR);
            if (err < pickRingTh && err < bestVal) { bestVal = err; best = h; }
        };

        tryAxis(Handle::MoveX, X);
        tryAxis(Handle::MoveY, Y);
        tryAxis(Handle::MoveZ, Z);

        tryRing(Handle::RotX, X);
        tryRing(Handle::RotY, Y);
        tryRing(Handle::RotZ, Z);

        return best;
    }

private:
    // ----- drag state -----
    Pose startTarget{};
    glm::vec3 dragPlaneN{0,1,0};
    float startAxisT = 0.0f;
    glm::vec3 startRingVec{1,0,0};

    void beginDrag(
        Handle h,
        const Ray& ray,
        const Pose& ee,
        const Pose& currentTarget,
        const glm::vec3& camForward,
        float ringR
    ){
        active = h;
        startTarget = currentTarget;

        glm::vec3 X = ee.rot * glm::vec3(1,0,0);
        glm::vec3 Y = ee.rot * glm::vec3(0,1,0);
        glm::vec3 Z = ee.rot * glm::vec3(0,0,1);

        auto axisDir = [&](Handle hh)->glm::vec3{
            if (hh == Handle::MoveX || hh == Handle::RotX) return X;
            if (hh == Handle::MoveY || hh == Handle::RotY) return Y;
            return Z;
        };

        glm::vec3 a = glm::normalize(axisDir(h));

        bool isMove = (h == Handle::MoveX || h == Handle::MoveY || h == Handle::MoveZ);
        if (isMove) {
            // Plane contains axis and is as "camera-facing" as possible.
            dragPlaneN = glm::normalize(glm::cross(a, glm::cross(camForward, a)));

            // Fallback if degenerate (camera looking nearly parallel to axis)
            if (glm::length(dragPlaneN) < 1e-6f) {
                glm::vec3 fallback = std::fabs(a.x) < 0.9f ? glm::vec3(1,0,0) : glm::vec3(0,1,0);
                dragPlaneN = glm::normalize(glm::cross(a, fallback));
            }

            auto q0 = rayIntersectPlanePoint(ray, ee.pos, dragPlaneN);
            startAxisT = q0 ? glm::dot(*q0 - ee.pos, a) : 0.0f;
        } else {
            // Ring plane normal = axis
            auto q0 = rayIntersectPlanePoint(ray, ee.pos, a);
            if (q0) {
                glm::vec3 v = *q0 - ee.pos;
                if (glm::length(v) < 1e-6f) v = (ee.rot * glm::vec3(0,1,0)) * ringR;
                startRingVec = glm::normalize(v);
            } else {
                startRingVec = ee.rot * glm::vec3(0,1,0);
            }
        }
    }

    void updateDrag(const Ray& ray, const Pose& ee, Pose& inOutTarget, float /*ringR*/){
        if (active == Handle::None) return;

        glm::vec3 X = ee.rot * glm::vec3(1,0,0);
        glm::vec3 Y = ee.rot * glm::vec3(0,1,0);
        glm::vec3 Z = ee.rot * glm::vec3(0,0,1);

        auto axisDir = [&](Handle hh)->glm::vec3{
            if (hh == Handle::MoveX || hh == Handle::RotX) return X;
            if (hh == Handle::MoveY || hh == Handle::RotY) return Y;
            return Z;
        };
        glm::vec3 a = glm::normalize(axisDir(active));

        bool isMove = (active == Handle::MoveX || active == Handle::MoveY || active == Handle::MoveZ);
        if (isMove) {
            auto q = rayIntersectPlanePoint(ray, ee.pos, dragPlaneN);
            if (!q) return;

            float t  = glm::dot(*q - ee.pos, a);
            float dt = (t - startAxisT);

            inOutTarget.pos = startTarget.pos + a * dt;
            inOutTarget.rot = startTarget.rot;
        } else {
            auto q = rayIntersectPlanePoint(ray, ee.pos, a);
            if (!q) return;

            glm::vec3 v = *q - ee.pos;
            if (glm::length(v) < 1e-6f) return;

            glm::vec3 v1 = glm::normalize(v);

            float sgn   = glm::dot(a, glm::cross(startRingVec, v1));
            float c     = glm::dot(startRingVec, v1);
            float angle = std::atan2(sgn, c);

            glm::quat dq = glm::angleAxis(angle, a);
            inOutTarget.rot = glm::normalize(dq * startTarget.rot);
            inOutTarget.pos = startTarget.pos;
        }
    }

    void endDrag() { active = Handle::None; }

private:
    template <typename DrawLineFn>
    void drawRing(
        const glm::vec3& center,
        const glm::vec3& normal,
        float r,
        const glm::vec3& color,
        DrawLineFn&& drawLine
    ) const {
        glm::vec3 n = glm::normalize(normal);

        // tangent basis (u,v) in ring plane
        glm::vec3 t = (std::fabs(n.x) < 0.9f) ? glm::vec3(1,0,0) : glm::vec3(0,1,0);
        glm::vec3 u = glm::normalize(glm::cross(n, t));
        glm::vec3 v = glm::normalize(glm::cross(n, u));

        int N = (s.ringSegments < 12) ? 12 : s.ringSegments;

        glm::vec3 prev = center + r * u;

        for (int i = 1; i <= N; ++i) {
            float a = (float)i / (float)N * 2.0f * 3.1415926535f;
            glm::vec3 p = center + r * (u * std::cos(a) + v * std::sin(a));
            drawLine(prev, p, color);
            prev = p;
        }
    }
};

} // namespace ImGuizmoLite
