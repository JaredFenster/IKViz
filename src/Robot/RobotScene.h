#pragma once
#include "URDFRobot.h"
#include <glm/glm.hpp>

class Shader;

class RobotScene {
public:
    bool LoadURDF(const std::string& urdfPath, const std::string& meshesRoot);
    void Draw(const Shader& shader);
    // Draw only the filled triangles (no edges), used for translucent preview
    void DrawPreview(const Shader& shader);
    // Draw only the feature edges (lines) - drawn opaque on top of previews
    void DrawEdges(const Shader& shader);

    // expose joint control (ImGui sliders)
    URDFRobot& Robot() { return robot_; }
    URDFRobot robot_;
    
private:
    void DrawLinkRecursive(const std::string& linkName,
                           const glm::mat4& parentWorld,
                           const Shader& shader);

};
