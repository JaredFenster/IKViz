#pragma once
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

class OrbitCamera {
public:
    void attachToWindow(GLFWwindow* window);

    void updateFromInput(bool imguiCapturingMouse);

    glm::mat4 view() const;
    glm::mat4 orthoProjFromRadius(GLFWwindow* window) const;

    void resetTarget() { target_ = glm::vec3(0.0f); }

    const glm::vec3& target() const { return target_; }
    float radius() const { return radius_; }
    glm::vec3 getCamPos() const;

private:
    static void scrollCallback(GLFWwindow* window, double xoff, double yoff);

private:
    GLFWwindow* window_ = nullptr;

    bool orbiting_ = false;
    bool panning_ = false;

    double lastX_ = 0, lastY_ = 0;
    double panLastX_ = 0, panLastY_ = 0;

    float yaw_ = 0.7854f;
    float pitch_ = 0.35f;
    float radius_ = 3.0f;

    glm::vec3 target_ = glm::vec3(0);
};
