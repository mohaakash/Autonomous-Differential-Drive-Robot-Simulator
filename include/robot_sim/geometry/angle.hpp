#pragma once

#include <cmath>
#include <stdexcept>

namespace robot_sim {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kTwoPi = 2.0 * kPi;

inline double wrap_to_pi(double angle) {
    if (!std::isfinite(angle)) {
        throw std::invalid_argument("angle must be finite");
    }

    double wrapped = std::remainder(angle, kTwoPi);
    if (wrapped >= kPi) {
        wrapped -= kTwoPi;
    }
    return wrapped;
}

inline double shortest_angular_distance(double from, double to) {
    return wrap_to_pi(to - from);
}

}  // namespace robot_sim
