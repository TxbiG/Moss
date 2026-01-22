#ifndef MOSS_GRADIENT_H
#define MOSS_GRADIENT_H

#include <Moss/Core/Variants/Color.h>
#include <Moss/Core/TArray.h>
#include <Moss/Moss_stdinc.h>

class [[nodiscard]] Gradient {
private:
    struct GradientPoint {
        float offset;
        Color color;

        GradientPoint(float o, const Color& c) : offset(o), color(c) {}
    };

    TArray<GradientPoint> points;

public:
    // Add a point to the gradient
    void add_point(float offset, Color color) {
        points.emplace_back(offset, color);
        std::sort(points.begin(), points.end(), [](const GradientPoint& a, const GradientPoint& b) { return a.offset < b.offset; });
    }

    // Get the color at a specific point index
    Color get_color(int point) const {
        if (point < 0 || point >= points.size()) {
            // Handle out-of-bounds access, return a default color or throw an exception
            return Color(); // Return default color
        }
        return points[point].color;
    }

    // Get the offset at a specific point index
    float get_offset(int point) const {
        if (point < 0 || point >= points.size()) { return 0.0f; } // / Handle out-of-bounds access, return a default offset or throw an exception. Return default offset
        return points[point].offset;
    }

    // Get the number of points in the gradient
    int get_point_count() const {
        return points.size();
    }

    // Remove a point from the gradient
    void remove_point(int point) {
        if (point < 0 || point >= points.size()) { return; } // Handle out-of-bounds access, do nothing or throw an exception
        points.erase(points.begin() + point);
    }

    // Reverse the order of points in the gradient
    void reverse() { std::reverse(points.begin(), points.end()); }

    // Sample the color at a specific offset (0 to 1)
    Color sample(float offset) const { 
        if (points.empty()) { return Color(); } // Return default color if no points are defined

        // Clamp offset to [0, 1]
        offset = Maths::clamp(offset, 0.0f, 1.0f);

        // Handle edge cases
        if (offset <= points.front().offset) { return points.front().color; }
        if (offset >= points.back().offset) { return points.back().color; }

        // Find surrounding gradient points
        for (size_t i = 0; i < points.size() - 1; ++i) {
            const auto& left = points[i];
            const auto& right = points[i + 1];

            if (offset >= left.offset && offset <= right.offset) { return lerp(left.color, right.color, (offset - left.offset) / (right.offset - left.offset)); }
        }

        return Color();         // Should never reach here if above cases are handled
    }

    // Set the color at a specific point index
    void set_color(int point, Color color) {
        if (point < 0 || point >= points.size()) { return; } // Handle out-of-bounds access, do nothing or throw an exception
        points[point].color = color;
    }

    // Set the offset at a specific point index
    void set_offset(int point, float offset) {
        if (point < 0 || point >= points.size()) { return; } // Handle out-of-bounds access, do nothing or throw an exception
        points[point].offset = offset;
        std::sort(points.begin(), points.end(), [](const GradientPoint& a, const GradientPoint& b) { return a.offset < b.offset; });
    }
};

#endif // MOSS_GRADIENT_H
