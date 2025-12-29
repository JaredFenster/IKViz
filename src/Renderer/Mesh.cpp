#include "Mesh.h"

Mesh::~Mesh() { destroy(); }

Mesh::Mesh(Mesh&& o) noexcept {
    vao_ = o.vao_; vbo_ = o.vbo_; count_ = o.count_; mode_ = o.mode_;
    o.vao_ = 0; o.vbo_ = 0; o.count_ = 0;
}

Mesh& Mesh::operator=(Mesh&& o) noexcept {
    if (this == &o) return *this;
    destroy();
    vao_ = o.vao_; vbo_ = o.vbo_; count_ = o.count_; mode_ = o.mode_;
    o.vao_ = 0; o.vbo_ = 0; o.count_ = 0;
    return *this;
}

void Mesh::destroy() {
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    vbo_ = 0; vao_ = 0; count_ = 0;
}


Mesh Mesh::create(const std::vector<float>& data, GLenum mode) {
    Mesh m;
    m.mode_ = mode;
    m.count_ = (GLsizei)(data.size() / 6);

    glGenVertexArrays(1, &m.vao_);
    glGenBuffers(1, &m.vbo_);

    glBindVertexArray(m.vao_);
    glBindBuffer(GL_ARRAY_BUFFER, m.vbo_);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(data.size() * sizeof(float)), data.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return m;
}

Mesh Mesh::fromVertexColorLines(const std::vector<float>& data) {
    return create(data, GL_LINES);
}

Mesh Mesh::fromVertexColorTriangles(const std::vector<float>& data) {
    return create(data, GL_TRIANGLES);
}

void Mesh::drawLines() const {
    glBindVertexArray(vao_);
    glDrawArrays(GL_LINES, 0, count_);
    glBindVertexArray(0);
}

void Mesh::drawTriangles() const {
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, count_);
    glBindVertexArray(0);
}
