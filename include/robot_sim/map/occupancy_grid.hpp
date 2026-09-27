#pragma once

#include <filesystem>
#include <istream>
#include <optional>
#include <string>
#include <vector>

#include "robot_sim/geometry/vector2.hpp"
#include "robot_sim/map/grid_cell.hpp"

namespace robot_sim {

struct MapBounds {
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
};

class OccupancyGrid {
public:
    [[nodiscard]] static OccupancyGrid load_from_file(
        const std::filesystem::path& path
    );

    [[nodiscard]] static OccupancyGrid parse(
        std::istream& input,
        std::string source_name = "<stream>"
    );

    [[nodiscard]] int width() const { return width_; }
    [[nodiscard]] int height() const { return height_; }
    [[nodiscard]] double resolution_m() const { return resolution_m_; }
    [[nodiscard]] Vector2 origin() const { return origin_; }
    [[nodiscard]] MapBounds bounds() const;

    [[nodiscard]] bool is_inside(GridCell cell) const;

    // Out-of-bounds cells are treated as occupied for safety.
    [[nodiscard]] bool is_occupied(GridCell cell) const;
    [[nodiscard]] bool is_free(GridCell cell) const;

    [[nodiscard]] Vector2 cell_center(GridCell cell) const;
    [[nodiscard]] std::optional<GridCell> world_to_cell(Vector2 point) const;

    [[nodiscard]] std::optional<GridCell> start_marker() const {
        return start_marker_;
    }

    [[nodiscard]] std::optional<GridCell> goal_marker() const {
        return goal_marker_;
    }

private:
    OccupancyGrid(
        int width,
        int height,
        double resolution_m,
        Vector2 origin,
        std::vector<char> occupied,
        std::optional<GridCell> start_marker,
        std::optional<GridCell> goal_marker
    );

    [[nodiscard]] std::size_t index(GridCell cell) const;

    int width_;
    int height_;
    double resolution_m_;
    Vector2 origin_;
    std::vector<char> occupied_;
    std::optional<GridCell> start_marker_;
    std::optional<GridCell> goal_marker_;
};

}  // namespace robot_sim
