#include "Shader.h"
#include <stdexcept>

Shader::Shader(const char* vsSrc, const char* fsSrc) {
    GLuint vs = compile(GL_VERTEX_SHADER, vsSrc);
    GLuint fs = compile(GL_FRAGMENT_SHADER, fsSrc);
    program_ = link(vs, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);
}

Shader::~Shader() {
    if (program_) glDeleteProgram(program_);
}

GLuint Shader::compile(GLenum type, const char* src) {
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);

    GLint ok = 0;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetShaderiv(sh, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetShaderInfoLog(sh, len, nullptr, log.data());
        glDeleteShader(sh);
        throw std::runtime_error("Shader compile failed:\n" + log);
    }
    return sh;
}

GLuint Shader::link(GLuint vs, GLuint fs) {
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);

    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetProgramInfoLog(p, len, nullptr, log.data());
        glDeleteProgram(p);
        throw std::runtime_error("Program link failed:\n" + log);
    }
    return p;
}

void Shader::use() const { glUseProgram(program_); }

void Shader::setMat4(const char* name, const glm::mat4& m) const {
    GLint loc = glGetUniformLocation(program_, name);
    glUniformMatrix4fv(loc, 1, GL_FALSE, &m[0][0]);
}

void Shader::setFloat(const char* name, float v) const {
    GLint loc = glGetUniformLocation(program_, name);
    glUniform1f(loc, v);
}


void Shader::setBool(const char* name, bool v) const {
    GLint loc = glGetUniformLocation(program_, name);
    glUniform1i(loc, v ? 1 : 0);
}

void Shader::setVec3(const char* name, const glm::vec3& v) const {
    GLint loc = glGetUniformLocation(program_, name);
    glUniform3f(loc, v.x, v.y, v.z);
}
