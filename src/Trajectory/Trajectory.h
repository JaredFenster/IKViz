#include <glm/glm.hpp>
#include <glad/glad.h>

class Trajectory{
public:
    void GeneratePoints(const glm::vec3& currentPos, const glm::vec3& newPos, int density);
    glm::vec3 getPoint(int index) { return points.at(index); }
    int getNumPoints() { return points.size(); }
private:
    std::vector<glm::vec3> points;
};