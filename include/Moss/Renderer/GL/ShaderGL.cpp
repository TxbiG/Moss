#include <Moss/Renderer/GL/ShaderGL.h>
#include <Moss/Renderer/GL/glad.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

static std::string get_file_contents(const char* path)
{
    std::ifstream ifs(path, std::ios::in | std::ios::binary);
    if (!ifs)
        throw std::runtime_error(std::string("Failed to open file: ") + path);

    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

static void compileErrors(GLuint obj, const char* stage)
{
    GLint success = 0;
    if (strcmp(stage, "PROGRAM") == 0) {
        glGetProgramiv(obj, GL_LINK_STATUS, &success);
        if (!success) {
            char info[1024];
            glGetProgramInfoLog(obj, 1024, nullptr, info);
            printf("[Shader] LINK ERROR (%s):\n%s\n", stage, info);
        }
    } else {
        glGetShaderiv(obj, GL_COMPILE_STATUS, &success);
        if (!success) {
            char info[1024];
            glGetShaderInfoLog(obj, 1024, nullptr, info);
            printf("[Shader] COMPILE ERROR (%s):\n%s\n", stage, info);
        }
    }
}

ShaderGL::ShaderGL(const char* vertexSrc, const char* fragmentSrc, bool isRawGLSL) {
    std::string vsrc = isRawGLSL ? vertexSrc : get_file_contents(vertexSrc);
    std::string fsrc = isRawGLSL ? fragmentSrc : get_file_contents(fragmentSrc);

    // compile vertex ShaderGL
    mVertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    const char* vs  = vsrc.c_str();
    glShaderSource(mVertexShaderID, 1, &vs, nullptr);
    glCompileShader(mVertexShaderID);
    compileErrors(mVertexShaderID, "VERTEX");

    // compile fragment ShaderGL
    mFragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
    const char* fs = fsrc.c_str();
    glShaderSource(mFragmentShaderID, 1, &fs, nullptr);
    glCompileShader(mFragmentShaderID);
    compileErrors(mFragmentShaderID, "FRAGMENT");

    // link program
    mProgramID = glCreateProgram();
    glAttachShader(mProgramID, mVertexShaderID);
    glAttachShader(mProgramID, mFragmentShaderID);
    glLinkProgram(mProgramID);
    compileErrors(mProgramID, "PROGRAM");

    // delete ShaderGLs after linking
    glDeleteShader(mVertexShaderID);
    glDeleteShader(mFragmentShaderID);
    mVertexShaderID = mFragmentShaderID = 0;
}

ShaderGL::ShaderGL(const char* vertFile, const char* fragFile, const char* geomFile, bool isRawGLSL) {
    std::string vsrc = isRawGLSL ? vertFile : get_file_contents(vertFile);
    std::string fsrc = isRawGLSL ? fragFile : get_file_contents(fragFile);
    std::string gsrc = isRawGLSL ? geomFile : get_file_contents(geomFile);
    // compile VS
    mVertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    const char* vs  = vsrc.c_str();
    glShaderSource(mVertexShaderID, 1, &vs, nullptr);
    glCompileShader(mVertexShaderID);
    compileErrors(mVertexShaderID, "VERTEX");

    // compile FS
    mFragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
    const char* fs    = fsrc.c_str();
    glShaderSource(mFragmentShaderID, 1, &fs, nullptr);
    glCompileShader(mFragmentShaderID);
    compileErrors(mFragmentShaderID, "FRAGMENT");

    // compile GS
    mGeometryShaderID = glCreateShader(GL_GEOMETRY_SHADER);
    const char* gs    = gsrc.c_str();
    glShaderSource(mGeometryShaderID, 1, &gs, nullptr);
    glCompileShader(mGeometryShaderID);
    compileErrors(mGeometryShaderID, "GEOMETRY");

    // link program
    mProgramID = glCreateProgram();
    glAttachShader(mProgramID, mVertexShaderID);
    glAttachShader(mProgramID, mFragmentShaderID);
    glAttachShader(mProgramID, mGeometryShaderID);
    glLinkProgram(mProgramID);
    compileErrors(mProgramID, "PROGRAM");

    // delete ShaderGL objects
    glDeleteShader(mVertexShaderID);
    glDeleteShader(mFragmentShaderID);
    glDeleteShader(mGeometryShaderID);
    mVertexShaderID = mFragmentShaderID = mGeometryShaderID = 0;
}

ShaderGL::~ShaderGL() { if (mProgramID) glDeleteProgram(mProgramID); }

void ShaderGL::bind() { glUseProgram(mProgramID); }
void ShaderGL::unbind() { glUseProgram(0); }

void ShaderGL::SetUniform(const char *name, const float value) { GLint location = getUniformLocation(name); if (location != -1) glUniform1f(location, value); }
void ShaderGL::SetUniform(const char *name, const int value) { GLint location = getUniformLocation(name); if (location != -1) glUniform1i(location, value); }
void ShaderGL::SetUniform(const char* name, const Color& value) {GLint location = getUniformLocation(name); if (location != -1) glUniform4f(location, value.r, value.g, value.b, value.a);}
void ShaderGL::SetUniform(const char *name, const Float2 &value) { GLint location = getUniformLocation(name); if (location != -1) glUniform2f(location, value.x, value.y); }
void ShaderGL::SetUniform(const char *name, const Int2 &value) { GLint location = getUniformLocation(name); if (location != -1) glUniform2i(location, value.x, value.y); }
void ShaderGL::SetUniform(const char *name, const Float3 &value) { GLint location = getUniformLocation(name); if (location != -1) glUniform3f(location, value.x, value.y, value.z); }
void ShaderGL::SetUniform(const char *name, const Int3 &value) { GLint location = getUniformLocation(name); if (location != -1) glUniform3i(location, value.x, value.y, value.z);}
void ShaderGL::SetUniform(const char *name, const Float4 &value) { GLint location = getUniformLocation(name); if (location != -1) glUniform4f(location, value.x, value.y, value.z, value.w);  }
void ShaderGL::SetUniform(const char *name, const Int4 &value) { GLint location = getUniformLocation(name); if (location != -1) glUniform4i(location, value.x, value.y, value.z, value.w);  }

//void ShaderGL::SetUniformMat2x3(const char *name, const Mat2x3 &value) { GLint location = glGetUniformLocation(mProgramID, name); if (location != -1) glUniformMatrix2x3fv(location, 1, GL_FALSE, reinterpret_cast<const float*>(&value)); }
//void ShaderGL::SetUniformMat2x4(const char *name, const Mat2x4 &value) { GLint location = glGetUniformLocation(mProgramID, name); if (location != -1) glUniformMatrix2x4fv(location, 1, GL_FALSE, reinterpret_cast<const float*>(&value)); }
//void ShaderGL::SetUniformMat3(const char *name, const Mat3 &value) { GLint location = glGetUniformLocation(mProgramID, name); if (location != -1) glUniformMatrix3fv(location, 1, GL_FALSE, reinterpret_cast<const float*>(&value)); }
//void ShaderGL::SetUniformMat3x2(const char *name, const Mat3x2 &value) { GLint location = glGetUniformLocation(mProgramID, name); if (location != -1) glUniformMatrix3x2fv(location, 1, GL_FALSE, reinterpret_cast<const float*>(&value)); }
//void ShaderGL::SetUniformMat3x4(const char *name, const Mat3x4 &value) { GLint location = glGetUniformLocation(mProgramID, name); if (location != -1) glUniformMatrix3x4fv(location, 1, GL_FALSE, reinterpret_cast<const float*>(&value)); }
void ShaderGL::SetUniform(const char* name, const Mat44& value) { GLint location = glGetUniformLocation(mProgramID, name); if (location != -1) glUniformMatrix4fv(location, 1, GL_FALSE, reinterpret_cast<const float*>(&value)); }
//void ShaderGL::SetUniformMat4x2(const char *name, const Mat4x2 &value) { GLint location = glGetUniformLocation(mProgramID, name); if (location != -1) glUniformMatrix4x2fv(location, 1, GL_FALSE, reinterpret_cast<const float*>(&value)); }
//void ShaderGL::SetUniformMat4x3(const char *name, const Mat4x3& value) { GLint location = glGetUniformLocation(mProgramID, name); if (location != -1) glUniformMatrix4x3fv(location, 1, GL_FALSE, reinterpret_cast<const float*>(&value)); }

/*
void ShaderGL::SetUniformUint(const char *name, unsigned int value) { GLint location = getUniformLocation(name); if (location != -1) glUniform1ui(location, value); }
void ShaderGL::SetUniformUint2(const char *name, unsigned int value0, unsigned int value1) { GLint location = getUniformLocation(name); if (location != -1) glUniform2ui(location, value0, value1); }
void ShaderGL::SetUniformUint3(const char *name, unsigned int value0, unsigned int value1, unsigned int value2) { GLint location = getUniformLocation(name); if (location != -1) glUniform3ui(location, value0, value1, value2); }
void ShaderGL::SetUniformUint4(const char *name, unsigned int value0, unsigned int value1, unsigned int value2, unsigned int value3) { GLint location = getUniformLocation(name); if (location != -1) glUniform4ui(location, value0, value1, value2, value3); }
void ShaderGL::SetUniformInt(const char *name, int count, uint32 *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform1uiv(location, count, value); }
void ShaderGL::SetUniformInt2(const char *name, int count, uint32 *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform2uiv(location, count, value); }
void ShaderGL::SetUniformInt3(const char *name, int count, uint32 *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform3uiv(location, count, value); }
void ShaderGL::SetUniformInt4(const char *name, int count, uint32 *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform4uiv(location, count, value); }
void ShaderGL::SetUniformArrayf(const char *name, int count, const float *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform1fv(location, count, value); }
void ShaderGL::SetUniformArrayf2(const char *name, int count, const float *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform2fv(location, count, value); }
void ShaderGL::SetUniformArrayf3(const char *name, int count, const float *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform3fv(location, count, value); }
void ShaderGL::SetUniformArrayf4(const char *name, int count, const float *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform4fv(location, count, value); }
void ShaderGL::SetUniformArrayi(const char *name, int count, const int *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform1iv(location, count, value); }
void ShaderGL::SetUniformArrayi2(const char *name, int count, const int *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform2iv(location, count, value); }
void ShaderGL::SetUniformArrayi3(const char *name, int count, const int *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform3iv(location, count, value); }
void ShaderGL::SetUniformArrayi4(const char *name, int count, const int *value) { GLint location = getUniformLocation(name); if (location != -1) glUniform4iv(location, count, value); }*/