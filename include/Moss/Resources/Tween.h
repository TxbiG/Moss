#ifndef MOSS_TWEEEN_H
#define MOSS_TWEEEN_H

#include <functional>
#include <cmath>
#include <algorithm>


enum class TweenProcessMode {TWEEN_PROCESS_PHYSICS, TWEEN_PROCESS_IDLE, };

enum class TweenPauseMode {
		TWEEN_PAUSE_BOUND,
		TWEEN_PAUSE_STOP,
		TWEEN_PAUSE_PROCESS,
};

enum class TweenTransitionType {
		TRANS_LINEAR,
		TRANS_SINE,
		TRANS_QUINT,
		TRANS_QUART,
		TRANS_QUAD,
		TRANS_EXPO,
		TRANS_ELASTIC,
		TRANS_CUBIC,
		TRANS_CIRC,
		TRANS_BOUNCE,
		TRANS_BACK,
		TRANS_SPRING,
		TRANS_MAX
};

enum class TweenEaseType { EASE_IN, EASE_OUT, EASE_IN_OUT, EASE_OUT_IN, EASE_MAX };

template <typename T>
class [[nodiscard]] Tween {
public:

    template <typename PropType>
    void tween_property(T* object, PropType T::* property, const PropType& target, float duration, TweenTransitionType trans = TRANS_LINEAR, TweenEaseType ease = EASE_IN_OUT)
    {
        m_object = object;
        m_update = [=, this](float eased) mutable {
            PropType start = object->*property;
            PropType value = lerp(start, target, eased);
            object->*property = value;
        };

        m_start_any = [=]() -> std::any { return object->*property; };
        m_target_any = target;
        m_duration = duration;
        m_elapsed = 0.0f;
        m_finished = false;

        m_transitionType = trans;
        m_easeType = ease;
    }

    void update(float delta) {
        if (m_finished || !m_update) return;

        m_elapsed += delta;
        float t = std::clamp(m_elapsed / m_duration, 0.0f, 1.0f);
        float eased = apply_ease(t);

        m_update(eased);

        if (t >= 1.0f) m_finished = true;
    }

    bool finished() const { return m_finished; }

private:
    // Generic lerp (works for scalars, colors, vectors, etc.)
    template <typename U>
    static U lerp(const U& a, const U& b, float t) {
        return a + (b - a) * t;
    }

    // Easings
    inline float Linear(float t) { return t; }
    inline float SineEaseIn(float t) { return 1 - std::cos((t * M_PI) / 2); }
    inline float SineEaseOut(float t) { return std::sin((t * M_PI) / 2); }
    inline float SineEaseInOut(float t) { return -(std::cos(M_PI * t) - 1) / 2; }

    inline float QuadEaseIn(float t) { return t * t; }
    inline float QuadEaseOut(float t) { return t * (2 - t); }
    inline float QuadEaseInOut(float t) { return (t < 0.5f) ? (2 * t * t) : (-1 + (4 - 2 * t) * t); }

    float apply_ease(float t) {
        switch (m_transitionType) {
            case TRANS_LINEAR: return Linear(t);
            case TRANS_SINE:
                switch (m_easeType) {
                    case EASE_IN: return SineEaseIn(t);
                    case EASE_OUT: return SineEaseOut(t);
                    case EASE_IN_OUT: return SineEaseInOut(t);
                    default: return Linear(t);
                }
            case TRANS_QUAD:
                switch (m_easeType) {
                    case EASE_IN: return QuadEaseIn(t);
                    case EASE_OUT: return QuadEaseOut(t);
                    case EASE_IN_OUT: return QuadEaseInOut(t);
                    default: return Linear(t);
                }
            default:
                return Linear(t);
        }
    }

    // Members
    T* m_object{nullptr};
    std::function<void(float)> m_update;

    std::any m_start_any;
    std::any m_target_any;

    float m_duration{1.0f};
    float m_elapsed{0.0f};
    bool m_finished{true};

    TweenProcessMode process_mode;
    TweenPauseMode pause_mode;
    TweenTransitionType default_transition;
    TweenEaseType default_ease;
    TweenTransitionType m_transitionType;
    TweenEaseType m_easeType;
};
#endif // MOSS_TWEEEN_H