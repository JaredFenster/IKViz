#pragma once
/*
  ImGuizmoLite.h (lightweight, header-only transform gizmo)

  FIXES:
  1) Drag no longer "outruns" the mouse:
     - Drag plane/axis/pivot are LOCKED at drag start (pivotPosStart/axisStart/planeNStart)
     - updateDrag uses those fixed values (does NOT use ee.pos which changes while dragging)

  2) Gizmo size is CONSTANT IN WORLD SPACE (not constant on screen):
     - Removed all pixel sizing + worldPerPixel scaling
     - Settings values are interpreted as WORLD UNITS directly
*/

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <cmath>
#include <optional>
#include <type_traits>
#include <algorithm>

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
    float axisLen        = 0.20f;  // length of translation axes
    float ringRadius     = 0.10f;  // radius of rotation rings

    float axisRadius     = 0.010f; // cylinder radius for axis
    float ringTubeRadius = 0.010f; // cylinder radius for ring tube

    // WORLD-SPACE pick thickness
    float pickAxisThickness = 0.03f;
    float pickRingThickness = 0.03f;

    // Ensure picking is at least some multiple of rendered radius
    float pickRadiusMultiplier = 2.0f;

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
        // WORLD-CONSTANT sizing
        axisLenW_     = s.axisLen;
        ringRW_       = s.ringRadius;
        axisRadW_     = s.axisRadius;
        ringTubeRadW_ = s.ringTubeRadius;

        pickAxisW_ = std::max(s.pickAxisThickness, axisRadW_ * s.pickRadiusMultiplier);
        pickRingW_ = std::max(s.pickRingThickness, ringTubeRadW_ * s.pickRadiusMultiplier);

        Ray ray = makeMouseRayWorld(mouseX, mouseY, viewportW, viewportH, view, proj);

        capturingMouse = (active != Handle::None);

        if (active == Handle::None) {
            hovered = hitTest(ray, ee, axisLenW_, ringRW_, pickAxisW_, pickRingW_);
        }

        if (lmbPressed && hovered != Handle::None) {
            beginDrag(hovered, ray, ee, target, camForward, ringRW_);
            capturingMouse = true;
        }

        if (active != Handle::None && lmbDown) {
            updateDrag(ray, ee, target, ringRW_);
            capturingMouse = true;
        }

        if (lmbReleased) {
            endDrag();
            capturingMouse = false;
        }
    }

    // Drawing (proj not needed for sizing anymore, kept for signature compatibility)
    template <typename DrawFn>
    void draw(const Pose& ee, const glm::mat4& /*proj*/, DrawFn&& drawFn) const {
        glm::vec3 X = ee.rot * glm::vec3(1,0,0);
        glm::vec3 Y = ee.rot * glm::vec3(0,1,0);
        glm::vec3 Z = ee.rot * glm::vec3(0,0,1);

        auto colFor = [&](Handle h, const glm::vec3& axisCol)->glm::vec3{
            if (active == h)  return s.colActive;
            if (hovered == h) return s.colHover;
            return axisCol;
        };

        // axes
        drawSegment(ee.pos, ee.pos + X * axisLenW_, axisRadW_, colFor(Handle::MoveX, s.colX), drawFn);
        drawSegment(ee.pos, ee.pos + Y * axisLenW_, axisRadW_, colFor(Handle::MoveY, s.colY), drawFn);
        drawSegment(ee.pos, ee.pos + Z * axisLenW_, axisRadW_, colFor(Handle::MoveZ, s.colZ), drawFn);

        // rings
        drawRingTube(ee.pos, X, ringRW_, ringTubeRadW_, colFor(Handle::RotX, s.ringUsesAxisColor ? s.colX : glm::vec3(1)), drawFn);
        drawRingTube(ee.pos, Y, ringRW_, ringTubeRadW_, colFor(Handle::RotY, s.ringUsesAxisColor ? s.colY : glm::vec3(1)), drawFn);
        drawRingTube(ee.pos, Z, ringRW_, ringTubeRadW_, colFor(Handle::RotZ, s.ringUsesAxisColor ? s.colZ : glm::vec3(1)), drawFn);
    }

private:
    // Cached world sizes (computed in update so draw/pick/drag match perfectly)
    float axisLenW_     = 0.30f;
    float ringRW_       = 0.18f;
    float axisRadW_     = 0.020f;
    float ringTubeRadW_ = 0.018f;
    float pickAxisW_    = 0.03f;
    float pickRingW_    = 0.03f;

    // ----- ray helpers -----

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
    float startAxisT = 0.0f;
    glm::vec3 startRingVec{1,0,0};

    // FIX: locked pivot/axis/plane for the whole drag
    glm::vec3 pivotPosStart{0,0,0};
    glm::vec3 axisStart{1,0,0};
    glm::vec3 planeNStart{0,1,0};

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

        // LOCK pivot at drag start (do NOT follow ee.pos while dragging)
        pivotPosStart = ee.pos;

        glm::vec3 X = ee.rot * glm::vec3(1,0,0);
        glm::vec3 Y = ee.rot * glm::vec3(0,1,0);
        glm::vec3 Z = ee.rot * glm::vec3(0,0,1);

        auto axisDir = [&](Handle hh)->glm::vec3{
            if (hh == Handle::MoveX || hh == Handle::RotX) return X;
            if (hh == Handle::MoveY || hh == Handle::RotY) return Y;
            return Z;
        };

        axisStart = glm::normalize(axisDir(h));

        bool isMove = (h == Handle::MoveX || h == Handle::MoveY || h == Handle::MoveZ);
        if (isMove) {
            // Plane contains axis and is as screen-facing as possible.
            planeNStart = glm::normalize(glm::cross(axisStart, glm::cross(camForward, axisStart)));

            // Fallback if degenerate (camera looking nearly parallel to axis)
            if (glm::length(planeNStart) < 1e-6f) {
                glm::vec3 fallback = (std::fabs(axisStart.x) < 0.9f) ? glm::vec3(1,0,0) : glm::vec3(0,1,0);
                planeNStart = glm::normalize(glm::cross(axisStart, fallback));
            }

            auto q0 = rayIntersectPlanePoint(ray, pivotPosStart, planeNStart);
            startAxisT = q0 ? glm::dot(*q0 - pivotPosStart, axisStart) : 0.0f;
        } else {
            // Ring plane normal = axis
            auto q0 = rayIntersectPlanePoint(ray, pivotPosStart, axisStart);
            if (q0) {
                glm::vec3 v = *q0 - pivotPosStart;
                if (glm::length(v) < 1e-6f) v = (ee.rot * glm::vec3(0,1,0)) * ringR;
                startRingVec = glm::normalize(v);
            } else {
                startRingVec = ee.rot * glm::vec3(0,1,0);
            }
        }
    }

    void updateDrag(const Ray& ray, const Pose& /*ee*/, Pose& inOutTarget, float /*ringR*/){
        if (active == Handle::None) return;

        bool isMove = (active == Handle::MoveX || active == Handle::MoveY || active == Handle::MoveZ);
        if (isMove) {
            // Use LOCKED pivot/plane/axis
            auto q = rayIntersectPlanePoint(ray, pivotPosStart, planeNStart);
            if (!q) return;

            float t  = glm::dot(*q - pivotPosStart, axisStart);
            float dt = (t - startAxisT);

            inOutTarget.pos = startTarget.pos + axisStart * dt;
            inOutTarget.rot = startTarget.rot;
        } else {
            // Use LOCKED pivot/axis
            auto q = rayIntersectPlanePoint(ray, pivotPosStart, axisStart);
            if (!q) return;

            glm::vec3 v = *q - pivotPosStart;
            float vlen = glm::length(v);
            if (vlen < 1e-6f) return;

            glm::vec3 v1 = v / vlen;

            float sgn   = glm::dot(axisStart, glm::cross(startRingVec, v1));
            float c     = glm::dot(startRingVec, v1);
            float angle = std::atan2(sgn, c);

            glm::quat dq = glm::angleAxis(angle, axisStart);
            inOutTarget.rot = glm::normalize(dq * startTarget.rot);
            inOutTarget.pos = startTarget.pos;
        }
    }

    void endDrag() { active = Handle::None; }

private:
    // ---- draw dispatch ----
    template <typename Fn>
    static constexpr bool HasDrawCylinder =
        std::is_invocable_v<Fn, const glm::vec3&, const glm::vec3&, float, const glm::vec3&>;

    template <typename Fn>
    static constexpr bool HasDrawLine =
        std::is_invocable_v<Fn, const glm::vec3&, const glm::vec3&, const glm::vec3&>;

    template <typename DrawFn>
    static void drawSegment(
        const glm::vec3& a,
        const glm::vec3& b,
        float radius,
        const glm::vec3& color,
        DrawFn&& drawFn
    ){
        if constexpr (HasDrawCylinder<DrawFn>) {
            drawFn(a, b, radius, color);
        } else if constexpr (HasDrawLine<DrawFn>) {
            (void)radius;
            drawFn(a, b, color);
        } else {
            static_assert(HasDrawLine<DrawFn> || HasDrawCylinder<DrawFn>,
                "DrawFn must be drawLine(p0,p1,color) or drawCylinder(p0,p1,radius,color)");
        }
    }

    template <typename DrawFn>
    void drawRingTube(
        const glm::vec3& center,
        const glm::vec3& normal,
        float r,
        float tubeRadius,
        const glm::vec3& color,
        DrawFn&& drawFn
    ) const {
        glm::vec3 n = glm::normalize(normal);

        glm::vec3 t = (std::fabs(n.x) < 0.9f) ? glm::vec3(1,0,0) : glm::vec3(0,1,0);
        glm::vec3 u = glm::normalize(glm::cross(n, t));
        glm::vec3 v = glm::normalize(glm::cross(n, u));

        int N = (s.ringSegments < 12) ? 12 : s.ringSegments;

        glm::vec3 prev = center + r * u;

        for (int i = 1; i <= N; ++i) {
            float a = (float)i / (float)N * 2.0f * 3.1415926535f;
            glm::vec3 p = center + r * (u * std::cos(a) + v * std::sin(a));

            drawSegment(prev, p, tubeRadius, color, drawFn);
            prev = p;
        }
    }
};

} // namespace ImGuizmoLite
