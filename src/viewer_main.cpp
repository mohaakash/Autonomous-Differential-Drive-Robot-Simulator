#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "raylib.h"

#include "robot_sim/collision/collision_checker.hpp"
#include "robot_sim/control/pure_pursuit.hpp"
#include "robot_sim/geometry/transform2d.hpp"
#include "robot_sim/kinematics/differential_drive.hpp"
#include "robot_sim/map/occupancy_grid.hpp"
#include "robot_sim/planning/a_star.hpp"
#include "robot_sim/sensors/lidar.hpp"
#include "robot_sim/simulation/fixed_step_simulator.hpp"

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 800;
constexpr int kPanelWidth = 300;
constexpr double kPi = 3.14159265358979323846;

struct ViewTransform {
    robot_sim::MapBounds bounds{};
    Rectangle viewport{};
    Rectangle map_rect{};
    float scale{1.0F};

    [[nodiscard]] ::Vector2 world_to_screen(robot_sim::Vector2 point) const {
        return {
            map_rect.x + static_cast<float>(point.x - bounds.min_x) * scale,
            map_rect.y + map_rect.height -
                static_cast<float>(point.y - bounds.min_y) * scale,
        };
    }

    [[nodiscard]] float metres_to_pixels(double metres) const {
        return static_cast<float>(metres) * scale;
    }
};

[[nodiscard]] ViewTransform make_view_transform(const robot_sim::OccupancyGrid& map) {
    const robot_sim::MapBounds bounds = map.bounds();
    const Rectangle viewport{
        0.0F,
        0.0F,
        static_cast<float>(kWindowWidth - kPanelWidth),
        static_cast<float>(kWindowHeight),
    };
    const float scale = std::min(
        (viewport.width - 80.0F) /
            static_cast<float>(bounds.max_x - bounds.min_x),
        (viewport.height - 80.0F) /
            static_cast<float>(bounds.max_y - bounds.min_y)
    );
    const Rectangle map_rect{
        viewport.x + (viewport.width -
                      static_cast<float>(bounds.max_x - bounds.min_x) * scale) / 2.0F,
        viewport.y + (viewport.height -
                      static_cast<float>(bounds.max_y - bounds.min_y) * scale) / 2.0F,
        static_cast<float>(bounds.max_x - bounds.min_x) * scale,
        static_cast<float>(bounds.max_y - bounds.min_y) * scale,
    };
    return {bounds, viewport, map_rect, scale};
}

void draw_map(
    const robot_sim::OccupancyGrid& map,
    const ViewTransform& view
) {
    DrawRectangleRec(view.viewport, Color{48, 88, 68, 255});
    DrawRectangleRec(
        {
            view.map_rect.x + 7.0F,
            view.map_rect.y + 9.0F,
            view.map_rect.width,
            view.map_rect.height,
        },
        Color{28, 54, 48, 120}
    );
    DrawRectangleRec(view.map_rect, Color{139, 187, 105, 255});

    const auto cell_rectangle = [&](int row, int column, double inset_m) {
        const robot_sim::Vector2 centre = map.cell_center({row, column});
        const double half = map.resolution_m() / 2.0 - inset_m;
        const ::Vector2 top_left = view.world_to_screen({
            centre.x - half,
            centre.y + half,
        });
        const ::Vector2 bottom_right = view.world_to_screen({
            centre.x + half,
            centre.y - half,
        });
        return Rectangle{
            top_left.x,
            top_left.y,
            bottom_right.x - top_left.x,
            bottom_right.y - top_left.y,
        };
    };

    for (int row = 0; row < map.height(); ++row) {
        for (int column = 0; column < map.width(); ++column) {
            const robot_sim::GridCell cell{row, column};
            if (!map.is_occupied(cell)) {
                const Color grass = ((row + column) % 2 == 0)
                    ? Color{145, 194, 111, 255}
                    : Color{139, 187, 105, 255};
                DrawRectangleRec(cell_rectangle(row, column, 0.012), grass);
                continue;
            }

            DrawRectangleRec(cell_rectangle(row, column, 0.0), Color{46, 79, 62, 255});
            DrawRectangleRec(cell_rectangle(row, column, 0.035), Color{61, 103, 69, 255});
            DrawRectangleLinesEx(
                cell_rectangle(row, column, 0.035),
                1.5F,
                Color{35, 67, 55, 220}
            );

            const ::Vector2 centre = view.world_to_screen(map.cell_center(cell));
            const float leaf_radius = std::max(
                2.0F,
                view.metres_to_pixels(map.resolution_m() * 0.13)
            );
            DrawCircleV(
                {centre.x - leaf_radius * 1.5F, centre.y - leaf_radius * 0.4F},
                leaf_radius,
                Color{85, 132, 76, 220}
            );
            DrawCircleV(
                {centre.x + leaf_radius * 1.3F, centre.y + leaf_radius * 0.5F},
                leaf_radius,
                Color{76, 119, 70, 220}
            );
        }
    }

    for (int column = 0; column <= map.width(); ++column) {
        const double x = map.origin().x +
            static_cast<double>(column) * map.resolution_m();
        const ::Vector2 start = view.world_to_screen({x, map.bounds().min_y});
        const ::Vector2 end = view.world_to_screen({x, map.bounds().max_y});
        DrawLineV(start, end, Color{233, 247, 190, 45});
    }
    for (int row = 0; row <= map.height(); ++row) {
        const double y = map.bounds().min_y +
            static_cast<double>(row) * map.resolution_m();
        const ::Vector2 start = view.world_to_screen({map.bounds().min_x, y});
        const ::Vector2 end = view.world_to_screen({map.bounds().max_x, y});
        DrawLineV(start, end, Color{233, 247, 190, 45});
    }

    DrawRectangleLinesEx(view.map_rect, 4.0F, Color{22, 54, 48, 255});
}

[[nodiscard]] std::uint32_t cell_hash(int row, int column) {
    std::uint32_t value = static_cast<std::uint32_t>(row + 101) * 0x45d9f3bu;
    value ^= static_cast<std::uint32_t>(column + 313) * 0x119de1f3u;
    value ^= value >> 16;
    value *= 0x7feb352du;
    value ^= value >> 15;
    return value;
}

[[nodiscard]] bool near_planned_path(
    robot_sim::Vector2 point,
    const robot_sim::PlannedPath& path,
    double distance_m
) {
    const double squared_distance = distance_m * distance_m;
    for (const robot_sim::Vector2 waypoint : path.waypoints) {
        if ((point - waypoint).squared_norm() <= squared_distance) {
            return true;
        }
    }
    return false;
}

void draw_tree(
    robot_sim::Vector2 world_position,
    double radius_m,
    const ViewTransform& view
) {
    const ::Vector2 centre = view.world_to_screen(world_position);
    const float radius = view.metres_to_pixels(radius_m);
    const float trunk_width = std::max(3.0F, radius * 0.28F);
    const float trunk_height = std::max(5.0F, radius * 0.75F);

    DrawCircleV(
        {centre.x + radius * 0.25F, centre.y + radius * 0.55F},
        radius * 0.95F,
        Color{29, 65, 48, 90}
    );
    DrawRectangleRec(
        {
            centre.x - trunk_width / 2.0F,
            centre.y + radius * 0.15F,
            trunk_width,
            trunk_height,
        },
        Color{112, 75, 48, 255}
    );
    DrawCircleV(
        {centre.x - radius * 0.50F, centre.y + radius * 0.10F},
        radius * 0.70F,
        Color{44, 116, 68, 255}
    );
    DrawCircleV(
        {centre.x + radius * 0.42F, centre.y + radius * 0.08F},
        radius * 0.73F,
        Color{57, 137, 73, 255}
    );
    DrawCircleV(
        {centre.x, centre.y - radius * 0.40F},
        radius * 0.76F,
        Color{78, 157, 78, 255}
    );
    DrawCircleV(
        {centre.x - radius * 0.25F, centre.y - radius * 0.60F},
        radius * 0.23F,
        Color{139, 198, 95, 210}
    );
}

void draw_bush(
    robot_sim::Vector2 world_position,
    double radius_m,
    const ViewTransform& view
) {
    const ::Vector2 centre = view.world_to_screen(world_position);
    const float radius = view.metres_to_pixels(radius_m);
    DrawCircleV(
        {centre.x + radius * 0.25F, centre.y + radius * 0.35F},
        radius * 1.05F,
        Color{32, 92, 61, 100}
    );
    DrawCircleV(
        {centre.x - radius * 0.45F, centre.y + radius * 0.05F},
        radius * 0.75F,
        Color{44, 131, 72, 255}
    );
    DrawCircleV(
        {centre.x + radius * 0.35F, centre.y - radius * 0.10F},
        radius * 0.82F,
        Color{69, 157, 76, 255}
    );
}

void draw_playground_decorations(
    const robot_sim::OccupancyGrid& map,
    const robot_sim::PlannedPath& path,
    const ViewTransform& view
) {
    for (int row = 0; row < map.height(); ++row) {
        for (int column = 0; column < map.width(); ++column) {
            const robot_sim::GridCell cell{row, column};
            if (map.is_occupied(cell)) {
                continue;
            }
            const robot_sim::Vector2 centre = map.cell_center(cell);
            if (map.start_marker() == cell || map.goal_marker() == cell ||
                near_planned_path(centre, path, 0.34)) {
                continue;
            }

            const std::uint32_t hash = cell_hash(row, column);
            if (hash % 11u == 0u) {
                draw_tree(centre, 0.125 + static_cast<double>(hash % 4u) * 0.012, view);
            } else if (hash % 5u == 0u) {
                draw_bush(centre, 0.095, view);
            } else if (hash % 3u == 0u) {
                const ::Vector2 screen = view.world_to_screen(centre);
                const float size = std::max(3.0F, view.metres_to_pixels(0.035));
                DrawLineV(
                    {screen.x - size, screen.y + size},
                    {screen.x - size * 0.15F, screen.y - size},
                    Color{54, 126, 70, 210}
                );
                DrawLineV(
                    {screen.x + size * 0.15F, screen.y + size},
                    {screen.x + size, screen.y - size * 0.6F},
                    Color{72, 147, 74, 210}
                );
            }
        }
    }
}

void draw_path(
    const robot_sim::PlannedPath& path,
    const ViewTransform& view
) {
    for (std::size_t index = 1; index < path.waypoints.size(); ++index) {
        const ::Vector2 start = view.world_to_screen(path.waypoints[index - 1]);
        const ::Vector2 end = view.world_to_screen(path.waypoints[index]);
        DrawLineEx(
            start,
            end,
            std::max(8.0F, view.metres_to_pixels(0.16)),
            Color{35, 72, 57, 170}
        );
        DrawLineEx(
            start,
            end,
            std::max(5.0F, view.metres_to_pixels(0.105)),
            Color{235, 135, 62, 255}
        );
        DrawLineEx(
            start,
            end,
            std::max(2.0F, view.metres_to_pixels(0.035)),
            Color{255, 210, 101, 255}
        );
    }
    for (std::size_t index = 0; index < path.waypoints.size(); index += 2) {
        DrawCircleV(
            view.world_to_screen(path.waypoints[index]),
            std::max(2.0F, view.metres_to_pixels(0.035)),
            Color{255, 225, 132, 235}
        );
    }
}

void draw_trace(
    const robot_sim::SimulationResult& result,
    std::size_t sample_index,
    const ViewTransform& view
) {
    if (result.trace.empty()) {
        return;
    }
    const std::size_t last = std::min(sample_index, result.trace.size() - 1);
    for (std::size_t index = 1; index <= last; ++index) {
        const ::Vector2 start = view.world_to_screen(result.trace[index - 1].pose.position());
        const ::Vector2 end = view.world_to_screen(result.trace[index].pose.position());
        DrawLineEx(
            start,
            end,
            std::max(7.0F, view.metres_to_pixels(0.09)),
            Color{26, 71, 74, 190}
        );
        DrawLineEx(
            start,
            end,
            std::max(4.0F, view.metres_to_pixels(0.052)),
            Color{54, 190, 166, 255}
        );
        DrawLineEx(
            start,
            end,
            std::max(1.0F, view.metres_to_pixels(0.018)),
            Color{171, 246, 190, 255}
        );
    }
}

void draw_lidar(
    const robot_sim::Pose2D& pose,
    const robot_sim::LidarParameters& parameters,
    const robot_sim::LaserScan& scan,
    const ViewTransform& view
) {
    const robot_sim::Transform2D world_from_lidar =
        robot_sim::Transform2D::from_pose(pose) *
        robot_sim::Transform2D(parameters.mount_translation, parameters.mount_yaw_rad);
    const ::Vector2 origin = view.world_to_screen(world_from_lidar.translation);
    const std::size_t stride = std::max<std::size_t>(1, scan.ranges_m.size() / 45);
    for (std::size_t index = 0; index < scan.ranges_m.size(); index += stride) {
        const double world_angle = world_from_lidar.yaw + scan.angle_at(index);
        const robot_sim::Vector2 endpoint{
            world_from_lidar.translation.x +
                scan.ranges_m[index] * std::cos(world_angle),
            world_from_lidar.translation.y +
                scan.ranges_m[index] * std::sin(world_angle),
        };
        DrawLineEx(
            origin,
            view.world_to_screen(endpoint),
            1.0F,
            scan.has_return[index] != 0
                ? Color{69, 123, 157, 150}
                : Color{69, 123, 157, 55}
        );
    }
}

[[nodiscard]] robot_sim::Vector2 local_to_world(
    const robot_sim::Pose2D& pose,
    robot_sim::Vector2 local_point
) {
    const double cosine = std::cos(pose.theta);
    const double sine = std::sin(pose.theta);
    return {
        pose.x + local_point.x * cosine - local_point.y * sine,
        pose.y + local_point.x * sine + local_point.y * cosine,
    };
}

[[nodiscard]] ::Vector2 local_to_screen(
    const robot_sim::Pose2D& pose,
    robot_sim::Vector2 local_point,
    const ViewTransform& view
) {
    return view.world_to_screen(local_to_world(pose, local_point));
}

template <std::size_t PointCount>
void draw_oriented_polygon(
    const robot_sim::Pose2D& pose,
    const std::array<robot_sim::Vector2, PointCount>& local_points,
    const ViewTransform& view,
    Color fill,
    Color outline
) {
    std::array<::Vector2, PointCount> screen_points{};
    for (std::size_t index = 0; index < PointCount; ++index) {
        screen_points[index] = local_to_screen(pose, local_points[index], view);
    }
    DrawTriangleFan(
        screen_points.data(),
        static_cast<int>(PointCount),
        fill
    );
    for (std::size_t index = 0; index < PointCount; ++index) {
        DrawLineV(
            screen_points[index],
            screen_points[(index + 1) % PointCount],
            outline
        );
    }
}

void draw_robot(
    const robot_sim::Pose2D& pose,
    double radius_m,
    const ViewTransform& view
) {
    const ::Vector2 centre = view.world_to_screen(pose.position());
    const float footprint_radius = view.metres_to_pixels(radius_m);
    DrawCircleLines(
        static_cast<int>(centre.x),
        static_cast<int>(centre.y),
        footprint_radius,
        Color{38, 70, 83, 70}
    );

    constexpr double body_length_m = 0.42;
    constexpr double body_width_m = 0.28;
    constexpr double half_body_length_m = body_length_m / 2.0;
    constexpr double half_body_width_m = body_width_m / 2.0;
    constexpr double wheel_length_m = 0.18;
    constexpr double wheel_width_m = 0.045;

    for (const double wheel_y : std::array<double, 2>{
             -half_body_width_m - wheel_width_m / 2.0,
             half_body_width_m + wheel_width_m / 2.0,
         }) {
        draw_oriented_polygon(
            pose,
            std::array<robot_sim::Vector2, 4>{{
                {-wheel_length_m / 2.0, wheel_y - wheel_width_m / 2.0},
                {wheel_length_m / 2.0, wheel_y - wheel_width_m / 2.0},
                {wheel_length_m / 2.0, wheel_y + wheel_width_m / 2.0},
                {-wheel_length_m / 2.0, wheel_y + wheel_width_m / 2.0},
            }},
            view,
            Color{32, 38, 48, 255},
            Color{14, 18, 24, 255}
        );
    }

    draw_oriented_polygon(
        pose,
        std::array<robot_sim::Vector2, 8>{{
            {half_body_length_m, 0.065},
            {half_body_length_m - 0.055, half_body_width_m},
            {-half_body_length_m + 0.04, half_body_width_m},
            {-half_body_length_m, 0.075},
            {-half_body_length_m, -0.075},
            {-half_body_length_m + 0.04, -half_body_width_m},
            {half_body_length_m - 0.055, -half_body_width_m},
            {half_body_length_m, -0.065},
        }},
        view,
        Color{231, 111, 81, 255},
        Color{38, 70, 83, 255}
    );

    draw_oriented_polygon(
        pose,
        std::array<robot_sim::Vector2, 4>{{
            {-0.085, -0.095},
            {0.085, -0.075},
            {0.085, 0.075},
            {-0.085, 0.095},
        }},
        view,
        Color{42, 67, 84, 255},
        Color{197, 218, 225, 255}
    );

    DrawLineV(
        local_to_screen(pose, {-0.085, 0.0}, view),
        local_to_screen(pose, {0.085, 0.0}, view),
        Color{197, 218, 225, 180}
    );

    const float lamp_radius = std::max(2.0F, view.metres_to_pixels(0.018));
    for (const double side : std::array<double, 2>{-1.0, 1.0}) {
        DrawCircleV(
            local_to_screen(pose, {0.175, side * 0.052}, view),
            lamp_radius,
            Color{255, 226, 138, 255}
        );
        DrawCircleV(
            local_to_screen(pose, {-0.175, side * 0.052}, view),
            lamp_radius,
            Color{173, 54, 62, 255}
        );
    }
}

void draw_panel(
    const robot_sim::SimulationResult& result,
    std::size_t sample_index,
    double playback_rate,
    bool paused
) {
    const int panel_x = kWindowWidth - kPanelWidth;
    DrawRectangle(
        panel_x,
        0,
        kPanelWidth,
        kWindowHeight,
        Color{25, 39, 53, 255}
    );
    DrawRectangle(panel_x, 0, kPanelWidth, 108, Color{31, 58, 66, 255});
    DrawRectangle(panel_x, 106, kPanelWidth, 4, Color{239, 151, 73, 255});
    DrawRectangleRounded(
        {
            static_cast<float>(panel_x + 16),
            112.0F,
            static_cast<float>(kPanelWidth - 32),
            170.0F,
        },
        0.12F,
        8,
        Color{31, 51, 64, 255}
    );
    DrawRectangleRounded(
        {
            static_cast<float>(panel_x + 16),
            294.0F,
            static_cast<float>(kPanelWidth - 32),
            52.0F,
        },
        0.12F,
        8,
        Color{31, 51, 64, 255}
    );
    DrawRectangleRounded(
        {
            static_cast<float>(panel_x + 16),
            360.0F,
            static_cast<float>(kPanelWidth - 32),
            144.0F,
        },
        0.12F,
        8,
        Color{31, 51, 64, 255}
    );
    DrawRectangleRounded(
        {
            static_cast<float>(panel_x + 16),
            570.0F,
            static_cast<float>(kPanelWidth - 32),
            178.0F,
        },
        0.12F,
        8,
        Color{31, 51, 64, 255}
    );
    const std::string status = std::string(
        robot_sim::simulation_status_name(result.status)
    );
    DrawText("DIFFERENTIAL DRIVE", panel_x + 24, 32, 20, Color{244, 241, 234, 255});
    DrawText(TextFormat("status: %s", status.c_str()), panel_x + 24, 82, 22, Color{244, 162, 97, 255});
    DrawText(TextFormat("frame: %i / %i", static_cast<int>(sample_index), static_cast<int>(result.trace.size())), panel_x + 24, 130, 18, Color{220, 225, 230, 255});
    if (!result.trace.empty()) {
        const robot_sim::TraceSample& sample = result.trace[
            std::min(sample_index, result.trace.size() - 1)
        ];
        DrawText(TextFormat("time: %.2f s", sample.time_s), panel_x + 24, 162, 18, Color{220, 225, 230, 255});
        DrawText(TextFormat("goal error: %.3f m", sample.goal_position_error_m), panel_x + 24, 194, 18, Color{220, 225, 230, 255});
        DrawText(TextFormat("cross-track: %.3f m", sample.cross_track_error_m), panel_x + 24, 226, 18, Color{220, 225, 230, 255});
        DrawText(TextFormat("LiDAR safety: %s", sample.safety_blocked ? "blocked" : "clear"), panel_x + 24, 258, 18, Color{220, 225, 230, 255});
    }
    DrawText(TextFormat("playback: %.1fx %s", playback_rate, paused ? "(paused)" : ""), panel_x + 24, 310, 18, Color{220, 225, 230, 255});
    DrawText("VISUAL GUIDE", panel_x + 24, 378, 16, Color{180, 190, 200, 255});
    DrawRectangle(panel_x + 24, 405, 18, 6, Color{255, 210, 101, 255});
    DrawText("planned route", panel_x + 52, 398, 16, Color{220, 225, 230, 255});
    DrawRectangle(panel_x + 24, 433, 18, 6, Color{100, 225, 184, 255});
    DrawText("travelled trail", panel_x + 52, 426, 16, Color{220, 225, 230, 255});
    DrawRectangle(panel_x + 24, 461, 18, 6, Color{104, 181, 218, 220});
    DrawText("LiDAR scan", panel_x + 52, 454, 16, Color{220, 225, 230, 255});
    DrawCircle(panel_x + 33, 489, 7.0F, Color{69, 157, 76, 255});
    DrawText("trees / greenery", panel_x + 52, 482, 16, Color{220, 225, 230, 255});
    DrawText("SPACE  play / pause", panel_x + 24, 610, 17, Color{180, 190, 200, 255});
    DrawText("R      reset", panel_x + 24, 638, 17, Color{180, 190, 200, 255});
    DrawText("LEFT/RIGHT  step", panel_x + 24, 666, 17, Color{180, 190, 200, 255});
    DrawText("UP/DOWN  speed", panel_x + 24, 694, 17, Color{180, 190, 200, 255});
    DrawText("ESC    quit", panel_x + 24, 722, 17, Color{180, 190, 200, 255});
}

robot_sim::SimulationResult make_simulation(
    const robot_sim::OccupancyGrid& map,
    const robot_sim::PlannedPath& path,
    robot_sim::GridCell start_cell,
    robot_sim::GridCell goal_cell
) {
    const robot_sim::Vector2 start = map.cell_center(start_cell);
    const robot_sim::Vector2 goal = map.cell_center(goal_cell);
    const robot_sim::PurePursuitController controller({
        0.35, 0.12, 0.35, 0.50, 2.0, 2.0, 1.5, 0.10, 0.12,
    });
    const robot_sim::DifferentialDriveKinematics kinematics({0.05, 0.30, 12.0});
    const robot_sim::CollisionChecker collision_checker(map, {0.20});
    const robot_sim::FixedStepSimulator simulator(
        controller,
        kinematics,
        collision_checker,
        {0.02, 180.0}
    );
    return simulator.run(
        {start.x, start.y, 0.0},
        {goal.x, goal.y, 0.0},
        path
    );
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc > 2 || (argc == 2 && std::string(argv[1]) == "--help")) {
            std::cout << "Usage: robot_sim_viewer [MAP]\n"
                      << "Controls: Space pause, R reset, Left/Right step, "
                         "Up/Down speed, Escape quit.\n"
                      << "Default map: maps/garden_maze.map\n";
            return argc > 2 ? 2 : 0;
        }
        const std::filesystem::path map_path = argc == 2
            ? argv[1]
            : "maps/garden_maze.map";
        const robot_sim::OccupancyGrid map =
            robot_sim::OccupancyGrid::load_from_file(map_path);
        if (!map.start_marker().has_value() || !map.goal_marker().has_value()) {
            throw std::invalid_argument("viewer map must contain S and G markers");
        }
        const robot_sim::AStarPlanner planner(map, 0.20);
        const robot_sim::PlanResult plan = planner.plan_cells(
            *map.start_marker(),
            *map.goal_marker()
        );
        if (!plan.success()) {
            throw std::runtime_error(
                "viewer planning failed: " +
                std::string(robot_sim::plan_status_name(plan.status))
            );
        }
        const robot_sim::SimulationResult result = make_simulation(
            map,
            plan.path,
            *map.start_marker(),
            *map.goal_marker()
        );
        if (result.trace.empty()) {
            throw std::runtime_error("viewer simulation produced no trace samples");
        }

        robot_sim::LidarParameters lidar_parameters;
        lidar_parameters.angle_min_rad = -0.5 * kPi;
        lidar_parameters.angle_max_rad = 0.5 * kPi;
        lidar_parameters.ray_count = 181;
        lidar_parameters.noise_stddev_m = 0.0;
        robot_sim::LidarSensor lidar(map, lidar_parameters);
        const ViewTransform view = make_view_transform(map);

        InitWindow(kWindowWidth, kWindowHeight, "2D Differential-Drive Robot");
        SetTargetFPS(60);
        SetExitKey(KEY_ESCAPE);

        bool paused = false;
        double playback_rate = 1.0;
        double playback_time_s = 0.0;
        while (!WindowShouldClose()) {
            if (IsKeyPressed(KEY_SPACE)) {
                paused = !paused;
            }
            if (IsKeyPressed(KEY_R)) {
                playback_time_s = 0.0;
                paused = false;
            }
            if (IsKeyPressed(KEY_UP)) {
                playback_rate = std::min(8.0, playback_rate * 2.0);
            }
            if (IsKeyPressed(KEY_DOWN)) {
                playback_rate = std::max(0.25, playback_rate / 2.0);
            }
            if (IsKeyPressed(KEY_RIGHT)) {
                paused = true;
                playback_time_s = std::min(
                    result.trace.back().time_s,
                    playback_time_s + 0.02
                );
            }
            if (IsKeyPressed(KEY_LEFT)) {
                paused = true;
                playback_time_s = std::max(0.0, playback_time_s - 0.02);
            }
            if (!paused) {
                playback_time_s += static_cast<double>(GetFrameTime()) * playback_rate;
                if (playback_time_s >= result.trace.back().time_s) {
                    playback_time_s = result.trace.back().time_s;
                    paused = true;
                }
            }

            std::size_t sample_index = 0;
            while (sample_index + 1 < result.trace.size() &&
                   result.trace[sample_index + 1].time_s <= playback_time_s) {
                ++sample_index;
            }
            const robot_sim::TraceSample& sample = result.trace[sample_index];
            const robot_sim::LaserScan scan = lidar.measure(sample.pose, sample.time_s);

            BeginDrawing();
            ClearBackground(Color{48, 88, 68, 255});
            draw_map(map, view);
            draw_playground_decorations(map, plan.path, view);
            draw_path(plan.path, view);
            draw_trace(result, sample_index, view);
            draw_lidar(sample.pose, lidar_parameters, scan, view);
            draw_robot(sample.pose, 0.20, view);
            const ::Vector2 start_screen = view.world_to_screen(
                map.cell_center(*map.start_marker())
            );
            const ::Vector2 goal_screen = view.world_to_screen(
                map.cell_center(*map.goal_marker())
            );
            DrawCircleV(start_screen, 8.0F, Color{38, 70, 83, 255});
            DrawCircleV(goal_screen, 8.0F, Color{231, 111, 81, 255});
            draw_panel(result, sample_index, playback_rate, paused);
            EndDrawing();
        }

        CloseWindow();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 2;
    }
}
