#ifndef MOSS_RECT_H
#define MOSS_RECT_H

#include <Moss/Moss_stdinc.h>

struct MOSS_API [[nodiscard]] Rect { 
    Rect() = default;
    Rect(float x, float y, float width, float height) : x(x), y(y), width(width), height(height) {}

    float x, y, width, height; 
};
struct MOSS_API [[nodiscard]] Recti { 
    Recti() = default;
    Recti(int x, int y, int width, int height) : x(x), y(y), width(width), height(height) {}

    int x, y, width, height; 
};

#endif // MOSS_RECT_H