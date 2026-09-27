#include "robot_sim/map/occupancy_grid.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace robot_sim {
namespace {

[[noreturn]] void parse_error(
    const std::string& source,
    std::size_t line,
    const std::string& message
) {
    throw std::runtime_error(
        source + ": line " + std::to_string(line) + ": " + message
    );
}

void reject_extra_tokens(
    std::istringstream& line_stream,
    const std::string& source,
    std::size_t line
) {
    std::string extra;
    if (line_stream >> extra) {
        parse_error(source, line, "unexpected extra value '" + extra + "'");
    }
}

}  // namespace

OccupancyGrid OccupancyGrid::load_from_file(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open map file: " + path.string());
    }
    return parse(input, path.string());
}

OccupancyGrid OccupancyGrid::parse(
    std::istream& input,
    std::string source_name
) {
    bool have_resolution = false;
    bool have_origin = false;
    bool have_width = false;
    bool have_height = false;
    bool found_data = false;
    double resolution_m = 0.0;
    Vector2 origin{};
    int width = 0;
    int height = 0;
    std::vector<std::string> rows;
    std::optional<GridCell> start_marker;
    std::optional<GridCell> goal_marker;

    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (!found_data) {
            if (line.empty()) {
                continue;
            }
            if (line == "data") {
                if (!have_resolution || !have_origin || !have_width || !have_height) {
                    parse_error(
                        source_name,
                        line_number,
                        "data appears before a complete header"
                    );
                }
                found_data = true;
                continue;
            }

            std::istringstream line_stream(line);
            std::string key;
            line_stream >> key;
            if (key == "resolution") {
                if (have_resolution || !(line_stream >> resolution_m)) {
                    parse_error(source_name, line_number, "invalid resolution header");
                }
                reject_extra_tokens(line_stream, source_name, line_number);
                if (!std::isfinite(resolution_m) || resolution_m <= 0.0) {
                    parse_error(
                        source_name,
                        line_number,
                        "resolution must be finite and greater than zero"
                    );
                }
                have_resolution = true;
            } else if (key == "origin") {
                if (have_origin || !(line_stream >> origin.x >> origin.y)) {
                    parse_error(source_name, line_number, "invalid origin header");
                }
                reject_extra_tokens(line_stream, source_name, line_number);
                if (!origin.is_finite()) {
                    parse_error(source_name, line_number, "origin must be finite");
                }
                have_origin = true;
            } else if (key == "width") {
                if (have_width || !(line_stream >> width)) {
                    parse_error(source_name, line_number, "invalid width header");
                }
                reject_extra_tokens(line_stream, source_name, line_number);
                if (width <= 0) {
                    parse_error(source_name, line_number, "width must be greater than zero");
                }
                have_width = true;
            } else if (key == "height") {
                if (have_height || !(line_stream >> height)) {
                    parse_error(source_name, line_number, "invalid height header");
                }
                reject_extra_tokens(line_stream, source_name, line_number);
                if (height <= 0) {
                    parse_error(source_name, line_number, "height must be greater than zero");
                }
                have_height = true;
            } else {
                parse_error(source_name, line_number, "unknown header field '" + key + "'");
            }
            continue;
        }

        if (static_cast<int>(rows.size()) < height) {
            if (line.empty()) {
                parse_error(source_name, line_number, "map rows must not be empty");
            }
            if (static_cast<int>(line.size()) != width) {
                parse_error(
                    source_name,
                    line_number,
                    "map row has width " + std::to_string(line.size()) +
                        ", expected " + std::to_string(width)
                );
            }
            rows.push_back(line);
        } else if (!line.empty()) {
            parse_error(source_name, line_number, "unexpected data after the map rows");
        }
    }

    if (!found_data) {
        throw std::runtime_error(source_name + ": missing data marker");
    }
    if (static_cast<int>(rows.size()) != height) {
        throw std::runtime_error(
            source_name + ": expected " + std::to_string(height) +
            " map rows, found " + std::to_string(rows.size())
        );
    }

    std::vector<char> occupied(static_cast<std::size_t>(width * height), 0);
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
            const char symbol = rows[static_cast<std::size_t>(row)]
                [static_cast<std::size_t>(column)];
            const GridCell cell{row, column};
            switch (symbol) {
                case '.':
                    break;
                case '#':
                    occupied[static_cast<std::size_t>(row * width + column)] = 1;
                    break;
                case 'S':
                    if (start_marker.has_value()) {
                        throw std::runtime_error(
                            source_name + ": duplicate start marker at row " +
                            std::to_string(row)
                        );
                    }
                    start_marker = cell;
                    break;
                case 'G':
                    if (goal_marker.has_value()) {
                        throw std::runtime_error(
                            source_name + ": duplicate goal marker at row " +
                            std::to_string(row)
                        );
                    }
                    goal_marker = cell;
                    break;
                default:
                    throw std::runtime_error(
                        source_name + ": invalid map symbol '" +
                        std::string(1, symbol) + "' at row " +
                        std::to_string(row) + ", column " +
                        std::to_string(column)
                    );
            }
        }
    }

    return OccupancyGrid(
        width,
        height,
        resolution_m,
        origin,
        std::move(occupied),
        start_marker,
        goal_marker
    );
}

OccupancyGrid::OccupancyGrid(
    int width,
    int height,
    double resolution_m,
    Vector2 origin,
    std::vector<char> occupied,
    std::optional<GridCell> start_marker,
    std::optional<GridCell> goal_marker
)
    : width_(width),
      height_(height),
      resolution_m_(resolution_m),
      origin_(origin),
      occupied_(std::move(occupied)),
      start_marker_(start_marker),
      goal_marker_(goal_marker) {}

MapBounds OccupancyGrid::bounds() const {
    return {
        origin_.x,
        origin_.y,
        origin_.x + static_cast<double>(width_) * resolution_m_,
        origin_.y + static_cast<double>(height_) * resolution_m_,
    };
}

bool OccupancyGrid::is_inside(GridCell cell) const {
    return cell.row >= 0 && cell.row < height_ &&
           cell.column >= 0 && cell.column < width_;
}

bool OccupancyGrid::is_occupied(GridCell cell) const {
    if (!is_inside(cell)) {
        return true;
    }
    return occupied_[index(cell)] != 0;
}

bool OccupancyGrid::is_free(GridCell cell) const {
    return is_inside(cell) && !is_occupied(cell);
}

Vector2 OccupancyGrid::cell_center(GridCell cell) const {
    if (!is_inside(cell)) {
        throw std::out_of_range("grid cell is outside the map");
    }
    return {
        origin_.x + (static_cast<double>(cell.column) + 0.5) * resolution_m_,
        origin_.y +
            (static_cast<double>(height_ - cell.row) - 0.5) * resolution_m_,
    };
}

std::optional<GridCell> OccupancyGrid::world_to_cell(Vector2 point) const {
    if (!point.is_finite()) {
        throw std::invalid_argument("world point must be finite");
    }

    const MapBounds map_bounds = bounds();
    if (point.x < map_bounds.min_x || point.x >= map_bounds.max_x ||
        point.y < map_bounds.min_y || point.y >= map_bounds.max_y) {
        return std::nullopt;
    }

    const int column = static_cast<int>(
        std::floor((point.x - origin_.x) / resolution_m_)
    );
    const int row = height_ - 1 - static_cast<int>(
        std::floor((point.y - origin_.y) / resolution_m_)
    );
    const GridCell cell{row, column};
    if (!is_inside(cell)) {
        return std::nullopt;
    }
    return cell;
}

std::size_t OccupancyGrid::index(GridCell cell) const {
    return static_cast<std::size_t>(cell.row * width_ + cell.column);
}

}  // namespace robot_sim
