#include "OrbitCamera.h"
#include <glm/gtc/matrix_transform.hpp>

static OrbitCamera* g_cam = nullptr;

void OrbitCamera::attachToWindow(GLFWwindow* window) {
    window_ = window;
    g_cam = this;
    glfwSetScrollCallback(window_, OrbitCamera::scrollCallback);
}

void OrbitCamera::scrollCallback(GLFWwindow*, double, double yoff) {
    if (!g_cam) return;
    g_cam->radius_ *= (yoff > 0.0) ? 0.9f : 1.1f;
    if (g_cam->radius_ < 0.3f) g_cam->radius_ = 0.3f;
    if (g_cam->radius_ > 50.0f) g_cam->radius_ = 50.0f;
}

glm::vec3 OrbitCamera::getCamPos() const {
    glm::vec3 forward(cosf(yaw_) * cosf(pitch_), sinf(yaw_) * cosf(pitch_), sinf(pitch_));
    glm::vec3 camPos = target_ - forward * radius_;
    return camPos;
}

void OrbitCamera::updateFromInput(bool imguiCapturingMouse) {
    if (!window_) return;

    glm::vec3 up(0, 0, 1);
    glm::vec3 forward(cosf(yaw_) * cosf(pitch_), sinf(yaw_) * cosf(pitch_), sinf(pitch_));
    glm::vec3 camPos = target_ - forward * radius_;
    glm::vec3 camForward = glm::normalize(target_ - camPos);
    glm::vec3 camRight = glm::normalize(glm::cross(camForward, up));
    glm::vec3 camUp = glm::normalize(glm::cross(camRight, camForward));

    if (imguiCapturingMouse) {
        panning_ = orbiting_ = false;
        return;
    }

    // CTRL + middle = pan
    if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS &&
        glfwGetKey(window_, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
    {
        double x, y;
        glfwGetCursorPos(window_, &x, &y);

        if (!panning_) {
            panning_ = true;
            panLastX_ = x; panLastY_ = y;
        }

        double dx = x - panLastX_;
        double dy = y - panLastY_;
        panLastX_ = x; panLastY_ = y;

        float panSpeed = radius_ * 0.0025f;
        target_ += (-camRight * (float)dx + camUp * (float)dy) * panSpeed;
    }
    // middle = orbit
    else if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS)
    {
        double x, y;
        glfwGetCursorPos(window_, &x, &y);

        if (!orbiting_) {
            orbiting_ = true;
            lastX_ = x; lastY_ = y;
        }

        double dx = x - lastX_;
        double dy = y - lastY_;
        lastX_ = x; lastY_ = y;

        const float s = 0.005f;
        yaw_ += (float)dx * s;
        pitch_ += (float)dy * s;

        const float limit = 1.55334f;
        if (pitch_ > limit) pitch_ = limit;
        if (pitch_ < -limit) pitch_ = -limit;
    }
    else {
        panning_ = orbiting_ = false;
    }
}

glm::mat4 OrbitCamera::view() const {
    glm::vec3 up(0,0,1);
    glm::vec3 forward(cosf(yaw_) * cosf(pitch_), sinf(yaw_) * cosf(pitch_), sinf(pitch_));
    glm::vec3 camPos = target_ - forward * radius_;
    return glm::lookAt(camPos, target_, up);
}

glm::mat4 OrbitCamera::orthoProjFromRadius(GLFWwindow* window) const {
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    float aspect = (h == 0) ? 1.0f : (float)w / (float)h;

    float halfH = radius_;
    float halfW = radius_ * aspect;

    return glm::ortho(-halfW, halfW, -halfH, halfH, 200.0f, -200.0f);
}
