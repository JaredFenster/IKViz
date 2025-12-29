#pragma once

struct GLFWwindow;

class App {
public:
    App();
    ~App();
    void run();

private:
    void initGLFW();
    void createWindow();
    void initGLAD();
    void initOpenGLState();
    void initImGui();
    void shutdownImGui();

    void beginFrame();
    void endFrame();

private:
    GLFWwindow* window_ = nullptr;
};
