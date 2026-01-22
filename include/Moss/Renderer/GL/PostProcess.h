#pragma once

//#include <Moss/Renderer/GL/PipelineStateGL.h>
#include <Moss/Renderer/GL/ShaderGL.h>


struct PostProcess {
    PostProcess(int w, int h);
    ~PostProcess();

    void bind();
    void render(GLuint inputTexture);

    GLuint getOutputTexture() const { return fboTexture; }
private:
    int width, height;
    GLuint fbo = 0;
    GLuint fboTexture = 0;
    GLuint rbo = 0;
    GLuint vao = 0, vbo = 0;
    ShaderGL shader;
};


class PostProcessingPipeline {
public:
    void add(PostProcess* pass) { passes.push_back(pass); }
    void clear() { passes.clear(); }

    void apply(GLuint inputTexture) {
        GLuint currentTex = inputTexture;
        for (size_t i = 0; i < passes.size(); ++i) {
            if (i < passes.size() - 1)
                passes[i]->bind(); // Render to next FBO
            else
                glBindFramebuffer(GL_FRAMEBUFFER, 0); // Final output

            passes[i]->render(currentTex);
            currentTex = passes[i]->getOutputTexture();
        }
    }

    std::vector<PostProcess*> passes;
};