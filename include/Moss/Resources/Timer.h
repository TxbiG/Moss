#ifndef TIMER_H
#define TIMER_H

#include <functional>
#include <vector>
#include <algorithm>

class Timer {
public:
    Timer(float duration, bool repeat = false) : m_duration(duration), m_remaining(duration), m_repeat(repeat), m_running(true), m_paused(false) {}

    void update(float deltaTime) {
        if (!m_running || m_paused) return;

        m_remaining -= deltaTime;
        if (m_remaining <= 0.0f) {
            if (m_callback) m_callback();

            if (m_repeat) { m_remaining = m_duration; } // reset for repeat
            else { m_running = false; } // one-shot
        }
    }

    // Controls
    void stop() { m_running = false; }
    void reset() { m_remaining = m_duration; m_running = true; }
    void pause() { m_paused = true; }
    void resume() { m_paused = false; }

    // Queries
    bool isRunning() const { return m_running; }
    bool isPaused() const { return m_paused; }
    float getRemaining() const { return m_remaining; }

    void setCallback(std::function<void()> cb) { m_callback = cb; }

private:
    float m_duration;
    float m_remaining;
    bool m_repeat;
    bool m_running;
    bool m_paused;
    std::function<void()> m_callback;
};

class TimerManager {
public:
    int CreateTimer(float duration, bool repeat, std::function<void()> callback) {
        Timer timer(duration, repeat);
        timer.setCallback(callback);
        m_timers.push_back(std::move(timer));
        return static_cast<int>(m_timers.size()) - 1;
    }

    void update(float deltaTime) { for (auto& timer : m_timers) { timer.update(deltaTime); } }

    void stopTimer(int id) { if (validID(id)) m_timers[id].stop(); }
    void pauseTimer(int id) { if (validID(id)) m_timers[id].pause(); }
    void resumeTimer(int id) { if (validID(id)) m_timers[id].resume(); }

private:
    std::vector<Timer> m_timers;
    bool validID(int id) const { return id >= 0 && id < m_timers.size(); }
};

#endif // TIMER_H