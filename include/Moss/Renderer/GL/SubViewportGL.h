#ifndef MOSS_SUBVIEWPORT_GL_H
#define MOSS_SUBVIEWPORT_GL_H

#include <Moss/Renderer/GL/glad.h>
#include <Moss/Renderer/Camera2.h>
#include <Moss/Renderer/Camera3.h>

enum class CameraType { ViewPortCamera2D, ViewPortCamera3D };

struct [[nodiscard]] MOSS_API SubViewport {
    SubViewport(int width, int height, CameraType type) : m_width(width), m_height(height), currentCamera(type) {
        initFBO();
        initQuad();
    }

    ~SubViewport() {
        if (colorTexture) glDeleteTextures(1, &colorTexture);
        if (depthBuffer) glDeleteRenderbuffers(1, &depthBuffer);
        if (fbo) glDeleteFramebuffers(1, &fbo);
        if (quadVAO) glDeleteVertexArrays(1, &quadVAO);
        if (quadVBO) glDeleteBuffers(1, &quadVBO);
    }

    void update() {
        switch (currentCamera) {
            case CameraType::ViewPortCamera2D: camera2D.update(m_width, m_height); break;
            case CameraType::ViewPortCamera3D: camera3D.update(m_width, m_height); break;
            default: MOSS_ERROR("Must be a 2D or 3D camera."); break;
        }
    }

    void begin() {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, m_width, m_height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void end() {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void drawToScreen(GLuint shaderProgram) {
        glUseProgram(shaderProgram);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, colorTexture);
        glUniform1i(glGetUniformLocation(shaderProgram, "u_Texture"), 0);

        glBindVertexArray(quadVAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }

    void resize(int newWidth, int newHeight) {
        if (newWidth == m_width && newHeight == m_height) return;

        m_width = newWidth;
        m_height = newHeight;

        glDeleteTextures(1, &colorTexture);
        glDeleteRenderbuffers(1, &depthBuffer);
        glDeleteFramebuffers(1, &fbo);

        initFBO();
    }

    GLuint getColorTexture() const { return colorTexture; }
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }

    Mat44 getViewProjMatrix() const{
        switch (currentCamera) {
            case CameraType::ViewPortCamera2D: return camera2D.getViewProjectionMatrix();
            case CameraType::ViewPortCamera3D: return camera3D.getViewProjectionMatrix();
            default: MOSS_ERROR("Must be a 2D or 3D camera."); return Mat44::sIdentity();
        }
    }

private:
    // Framebuffer
    GLuint fbo = 0;
    GLuint colorTexture = 0;
    GLuint depthBuffer = 0;

    // Fullscreen quad
    GLuint quadVAO = 0;
    GLuint quadVBO = 0;

    int m_width = 0;
    int m_height = 0;

    CameraType currentCamera = CameraType::ViewPortCamera2D;
    Camera2 camera2D;
    Camera3 camera3D;

    void initFBO() {
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        glGenTextures(1, &colorTexture);
        glBindTexture(GL_TEXTURE_2D, colorTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_width, m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);  // For pixel-perfect upscaling
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTexture, 0);

        glGenRenderbuffers(1, &depthBuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, depthBuffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, m_width, m_height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            MOSS_ERROR("[SubViewport] Framebuffer not complete!");
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void initQuad() {
        float quadVertices[] = {
            // positions   // texCoords
            -1.0f,  1.0f,   0.0f, 1.0f,
            -1.0f, -1.0f,   0.0f, 0.0f,
             1.0f, -1.0f,   1.0f, 0.0f,

            -1.0f,  1.0f,   0.0f, 1.0f,
             1.0f, -1.0f,   1.0f, 0.0f,
             1.0f,  1.0f,   1.0f, 1.0f
        };

        glGenVertexArrays(1, &quadVAO);
        glGenBuffers(1, &quadVBO);
        glBindVertexArray(quadVAO);

        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);

        glEnableVertexAttribArray(0); // pos
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);

        glEnableVertexAttribArray(1); // texCoord
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

        glBindVertexArray(0);
    }
};

#endif // MOSS_SUBVIEWPORT_GL_H