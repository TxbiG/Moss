#pragma once

#include <Moss/Moss_stdinc.h>

MOSS_WARNINGS_BEGIN

struct MOSS_EXPORT_GCC_BUG_WORKAROUND [[nodiscard]] Rect { 
    Rect() = default;
    Rect(float x, float y, float width, float height) : x(x), y(y), width(width), height(height) {}

    float x, y, width, height; 
};
struct MOSS_EXPORT_GCC_BUG_WORKAROUND [[nodiscard]] Recti { 
    Recti() = default;
    Recti(int x, int y, int width, int height) : x(x), y(y), width(width), height(height) {}

    int x, y, width, height; 
};

MOSS_WARNINGS_END