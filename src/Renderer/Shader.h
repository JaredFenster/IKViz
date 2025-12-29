#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>

class Shader {
public:
    Shader(const char* vsSrc, const char* fsSrc);
    ~Shader();

    void use() const;
    GLuint id() const { return program_; }

    void setMat4(const char* name, const glm::mat4& m) const;
    void setFloat(const char* name, float v) const;

private:
    GLuint compile(GLenum type, const char* src);
    GLuint link(GLuint vs, GLuint fs);

    GLuint program_ = 0;
};
