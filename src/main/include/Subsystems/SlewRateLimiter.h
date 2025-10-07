#ifndef SLEWRATELIMITER_H
#define SLEWRATELIMITER_H

class SlewRateLimiter {
public:
    // rateLimit is in units per second; initialValue is the starting output.
    explicit SlewRateLimiter(double rateLimit, double initialValue = 0.0)
        : m_rateLimit(rateLimit), m_prevValue(initialValue) {}

    // dt: time interval in seconds between calls.
    double Calculate(double input, double dt) {
        double delta = input - m_prevValue;
        double maxDelta = m_rateLimit * dt;
        if (delta > maxDelta) {
            delta = maxDelta;
        } else if (delta < -maxDelta) {
            delta = -maxDelta;
        }
        m_prevValue += delta;
        return m_prevValue;
    }

private:
    double m_rateLimit;
    double m_prevValue;
};

#endif
