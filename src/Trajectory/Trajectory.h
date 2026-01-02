#include <glm/glm.hpp>
#include <glad/glad.h>

class Trajectory{
public:
    std::vector<glm::vec3> GeneratePoints(glm::vec3 currentPos, glm::vec3 newPos, int density);
private:
    std::vector<glm::vec3> points;
};