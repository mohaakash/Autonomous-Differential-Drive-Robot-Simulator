#pragma once

#include <algorithm>
#include <cmath>

namespace robot_sim {

struct AxisAlignedRectangle {
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};

    [[nodiscard]] bool is_valid() const {
        return std::isfinite(min_x) && std::isfinite(min_y) &&
               std::isfinite(max_x) && std::isfinite(max_y) &&
               min_x < max_x && min_y < max_y;
    }

    [[nodiscard]] bool contains(double x, double y) const {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
    }
};

}  // namespace robot_sim
