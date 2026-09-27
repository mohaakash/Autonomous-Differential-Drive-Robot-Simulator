#pragma once

#include <cmath>

namespace robot_sim {

struct Vector2 {
    double x{0.0};
    double y{0.0};

    constexpr Vector2() = default;
    constexpr Vector2(double x_value, double y_value) : x(x_value), y(y_value) {}

    [[nodiscard]] double squared_norm() const { return x * x + y * y; }
    [[nodiscard]] double norm() const { return std::hypot(x, y); }
    [[nodiscard]] bool is_finite() const {
        return std::isfinite(x) && std::isfinite(y);
    }
};

constexpr Vector2 operator+(Vector2 lhs, Vector2 rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

constexpr Vector2 operator-(Vector2 lhs, Vector2 rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

constexpr Vector2 operator-(Vector2 value) {
    return {-value.x, -value.y};
}

constexpr Vector2 operator*(Vector2 value, double scale) {
    return {value.x * scale, value.y * scale};
}

constexpr Vector2 operator*(double scale, Vector2 value) {
    return value * scale;
}

constexpr Vector2 operator/(Vector2 value, double scale) {
    return {value.x / scale, value.y / scale};
}

constexpr double dot(Vector2 lhs, Vector2 rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

inline Vector2 rotate(Vector2 point, double yaw) {
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    return {
        cosine * point.x - sine * point.y,
        sine * point.x + cosine * point.y,
    };
}

}  // namespace robot_sim
