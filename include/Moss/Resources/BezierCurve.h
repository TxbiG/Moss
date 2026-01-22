#pragma once

#include <Moss/Core/Variants/Vector/Float2.h>
#include <Moss/Core/Variants/Vector/Float3.h>

#include <vector>

class [[nodiscard]] BezierCurve {
public:
    void clear() { controlPoints.clear(); }

    void addControlPoint(const float& point) { controlPoints.push_back(point); }

    void setControlPoints(const std::vector<float>& points) { controlPoints = points; }

    // Evaluate using De Casteljau's algorithm
    float evaluate(float t) const
    {
        std::vector<float> temp = controlPoints;
        int n = static_cast<int>(temp.size());

        if (n == 0)
            return float(0);

        for (int j = 1; j < n; ++j)
            for (int i = 0; i < n - j; ++i)
                temp[i] = (1.0f - t) * temp[i] + t * temp[i + 1];

        return temp[0];
    }

    size_t getOrder() const { return controlPoints.size() - 1; }

    bool empty() const { return controlPoints.empty(); }
private:
    std::vector<float> controlPoints;
};



class [[nodiscard]] BezierCurve2 {
public:
    void clear() { controlPoints.clear(); }

    void addControlPoint(const Float2& point) { controlPoints.push_back(point); }

    void setControlPoints(const std::vector<Float2>& points) { controlPoints = points; }

    // Evaluate using De Casteljau's algorithm
    Float2 evaluate(float t) const
    {
        std::vector<Float2> temp = controlPoints;
        int n = static_cast<int>(temp.size());

        if (n == 0)
            return Float2(0);

        for (int j = 1; j < n; ++j)
            for (int i = 0; i < n - j; ++i)
                temp[i] = (1.0f - t) * temp[i] + t * temp[i + 1];

        return temp[0];
    }

    size_t getOrder() const { return controlPoints.size() - 1; }

    bool empty() const { return controlPoints.empty(); }
private:
    std::vector<Float2> controlPoints;
};


class [[nodiscard]] BezierCurve2
{
public:
    void clear() { controlPoints.clear(); }

    void addControlPoint(const Float3& point) { controlPoints.push_back(point); }

    void setControlPoints(const std::vector<Float3>& points) { controlPoints = points; }

    // Evaluate using De Casteljau's algorithm
    Float3 evaluate(float t) const
    {
        std::vector<Float3> temp = controlPoints;
        int n = static_cast<int>(temp.size());

        if (n == 0) { return Float3(0, 0, 0); }

        for (int j = 1; j < n; ++j)
            for (int i = 0; i < n - j; ++i)
                temp[i] = (1.0f - t) * temp[i] + t * temp[i + 1];

        return temp[0];
    }

    size_t getOrder() const { return controlPoints.size() - 1; }

    bool empty() const { return controlPoints.empty(); }
private:
    std::vector<Float3> controlPoints;
};