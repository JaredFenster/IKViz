#pragma once
#include <glad/glad.h>
#include <vector>

class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    static Mesh fromVertexColorLines(const std::vector<float>& interleavedPosColor);
    static Mesh fromVertexColorTriangles(const std::vector<float>& interleavedPosColor);

    void drawLines() const;
    void drawTriangles() const;
    
private:
    static Mesh create(const std::vector<float>& data, GLenum mode);
    void destroy();

    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLsizei count_ = 0;
    GLenum mode_ = GL_LINES;
};
