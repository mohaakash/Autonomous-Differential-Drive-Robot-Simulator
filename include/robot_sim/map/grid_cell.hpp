#pragma once

namespace robot_sim {

struct GridCell {
    int row{0};
    int column{0};
};

constexpr bool operator==(GridCell lhs, GridCell rhs) {
    return lhs.row == rhs.row && lhs.column == rhs.column;
}

constexpr bool operator!=(GridCell lhs, GridCell rhs) {
    return !(lhs == rhs);
}

}  // namespace robot_sim
