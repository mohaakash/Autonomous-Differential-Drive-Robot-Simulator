#include "robot_sim/experiments/experiment_runner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "robot_sim/geometry/angle.hpp"

namespace robot_sim {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct ScenarioRun {
    OccupancyGrid map;
    GridCell start_cell;
    GridCell goal_cell;
    PlannedPath planned_path;
    PlanStatus plan_status{PlanStatus::Unreachable};
    SimulationResult simulation;
    LidarParameters lidar_parameters;
};

[[nodiscard]] std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const char character : value) {
        switch (character) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped += character;
                break;
        }
    }
    return escaped;
}

[[nodiscard]] std::string yaml_quote(const std::string& value) {
    std::string quoted = "\"";
    for (const char character : value) {
        if (character == '\\' || character == '"') {
            quoted += '\\';
        }
        quoted += character;
    }
    quoted += '"';
    return quoted;
}

void ensure_stream(const std::ofstream& output, const std::filesystem::path& path) {
    if (!output) {
        throw std::runtime_error("cannot write artifact: " + path.string());
    }
}

void write_resolved_config(
    const ExperimentConfig& config,
    const ScenarioRun& run,
    const std::filesystem::path& path
) {
    std::ofstream output(path, std::ios::trunc);
    ensure_stream(output, path);
    output << "map_path: " << yaml_quote(config.map_path.string()) << '\n'
           << "random_seed: " << config.random_seed << '\n'
           << "robot_radius_m: " << config.robot_radius_m << '\n'
           << "simulation_dt_s: " << config.simulation_dt_s << '\n'
           << "maximum_duration_s: " << config.maximum_duration_s << '\n'
           << "safety_policy: disabled\n"
           << "start_cell: [" << run.start_cell.row << ", "
           << run.start_cell.column << "]\n"
           << "goal_cell: [" << run.goal_cell.row << ", "
           << run.goal_cell.column << "]\n"
           << "lidar:\n"
           << "  angle_min_rad: " << run.lidar_parameters.angle_min_rad << '\n'
           << "  angle_max_rad: " << run.lidar_parameters.angle_max_rad << '\n'
           << "  ray_count: " << run.lidar_parameters.ray_count << '\n'
           << "  range_min_m: " << run.lidar_parameters.range_min_m << '\n'
           << "  range_max_m: " << run.lidar_parameters.range_max_m << '\n'
           << "  noise_stddev_m: " << run.lidar_parameters.noise_stddev_m << '\n';
    ensure_stream(output, path);
}

[[nodiscard]] double completion_time_s(const SimulationResult& result) {
    if (result.trace.empty()) {
        return 0.0;
    }
    return result.trace.back().time_s;
}

[[nodiscard]] double mean_cross_track_error(const SimulationResult& result) {
    if (result.trace.empty()) {
        return 0.0;
    }
    double total = 0.0;
    for (const TraceSample& sample : result.trace) {
        total += sample.cross_track_error_m;
    }
    return total / static_cast<double>(result.trace.size());
}

[[nodiscard]] std::size_t collision_count(const SimulationResult& result) {
    return static_cast<std::size_t>(std::count_if(
        result.trace.begin(),
        result.trace.end(),
        [](const TraceSample& sample) { return sample.collision; }
    ));
}

void write_summary_json(
    const ExperimentConfig& config,
    const ScenarioRun& run,
    const std::filesystem::path& path
) {
    const SimulationResult& result = run.simulation;
    std::ofstream output(path, std::ios::trunc);
    ensure_stream(output, path);
    output << std::setprecision(17)
           << "{\n"
           << "  \"status\": \"" << simulation_status_name(result.status)
           << "\",\n"
           << "  \"plan_status\": \"" << plan_status_name(run.plan_status)
           << "\",\n"
           << "  \"success\": " << (result.status == SimulationStatus::GoalReached ? "true" : "false") << ",\n"
           << "  \"map_path\": \"" << json_escape(config.map_path.string()) << "\",\n"
           << "  \"random_seed\": " << config.random_seed << ",\n"
           << "  \"goal_position_error_m\": " << result.final_goal_position_error_m << ",\n"
           << "  \"final_heading_error_rad\": " << result.final_heading_error_rad << ",\n"
           << "  \"max_cross_track_error_m\": " << result.max_cross_track_error_m << ",\n"
           << "  \"mean_cross_track_error_m\": " << mean_cross_track_error(result) << ",\n"
           << "  \"path_length_m\": " << run.planned_path.length_m << ",\n"
           << "  \"travelled_distance_m\": " << result.travelled_distance_m << ",\n"
           << "  \"completion_time_s\": " << completion_time_s(result) << ",\n"
           << "  \"simulation_steps\": "
           << (result.trace.empty() ? 0U : result.trace.back().step) << ",\n"
           << "  \"collision_count\": " << collision_count(result) << ",\n"
           << "  \"replan_count\": " << result.replan_count << ",\n"
           << "  \"emergency_stop_count\": " << result.safety_stop_count << ",\n"
           << "  \"command_clip_count\": " << result.command_clip_count << ",\n"
           << "  \"mean_step_compute_ms\": null,\n"
           << "  \"max_step_compute_ms\": null,\n"
           << "  \"artifacts\": {\n"
           << "    \"resolved_config\": \"resolved_config.yaml\",\n"
           << "    \"summary\": \"summary.json\",\n"
           << "    \"trace\": \"trace.csv\",\n"
           << "    \"planned_path\": \"planned_path.csv\",\n"
           << "    \"lidar\": \"lidar.csv\",\n"
           << "    \"visualization\": \"map_overlay.svg\"\n"
           << "  }\n"
           << "}\n";
    ensure_stream(output, path);
}

void write_trace_csv(
    const SimulationResult& result,
    const std::filesystem::path& path
) {
    std::ofstream output(path, std::ios::trunc);
    ensure_stream(output, path);
    output << std::setprecision(17)
           << "step,time_s,x_m,y_m,theta_rad,requested_v_m_s,requested_omega_rad_s,"
              "executed_v_m_s,executed_omega_rad_s,left_wheel_rad_s,right_wheel_rad_s,"
              "goal_error_m,heading_error_rad,cross_track_error_m,selected_waypoint_index,"
              "command_limited,collision,safety_blocked,replanned\n";
    for (const TraceSample& sample : result.trace) {
        output << sample.step << ',' << sample.time_s << ','
               << sample.pose.x << ',' << sample.pose.y << ',' << sample.pose.theta << ','
               << sample.requested_twist.linear_x << ','
               << sample.requested_twist.angular_z << ','
               << sample.executed_twist.linear_x << ','
               << sample.executed_twist.angular_z << ','
               << sample.wheel_command.left_rad_s << ','
               << sample.wheel_command.right_rad_s << ','
               << sample.goal_position_error_m << ','
               << sample.heading_error_rad << ','
               << sample.cross_track_error_m << ','
               << sample.selected_waypoint_index << ','
               << (sample.command_limited ? 1 : 0) << ','
               << (sample.collision ? 1 : 0) << ','
               << (sample.safety_blocked ? 1 : 0) << ','
               << (sample.replanned ? 1 : 0) << '\n';
    }
    ensure_stream(output, path);
}

void write_planned_path_csv(
    const PlannedPath& path_data,
    const std::filesystem::path& path
) {
    std::ofstream output(path, std::ios::trunc);
    ensure_stream(output, path);
    output << std::setprecision(17) << "index,x_m,y_m\n";
    for (std::size_t index = 0; index < path_data.waypoints.size(); ++index) {
        const Vector2 waypoint = path_data.waypoints[index];
        output << index << ',' << waypoint.x << ',' << waypoint.y << '\n';
    }
    ensure_stream(output, path);
}

void write_lidar_csv(
    const ScenarioRun& run,
    const std::filesystem::path& path
) {
    LidarSensor sensor(run.map, run.lidar_parameters);
    std::ofstream output(path, std::ios::trunc);
    ensure_stream(output, path);
    output << std::setprecision(17)
           << "step,time_s,ray_index,angle_rad,range_m,has_return\n";
    for (const TraceSample& sample : run.simulation.trace) {
        const LaserScan scan = sensor.measure(sample.pose, sample.time_s);
        for (std::size_t index = 0; index < scan.ranges_m.size(); ++index) {
            output << sample.step << ',' << sample.time_s << ',' << index << ','
                   << scan.angle_at(index) << ',' << scan.ranges_m[index] << ','
                   << (scan.has_return[index] != 0 ? 1 : 0) << '\n';
        }
    }
    ensure_stream(output, path);
}

[[nodiscard]] int svg_x(
    const MapBounds& bounds,
    double canvas_width,
    double x
) {
    return static_cast<int>(std::lround(
        (x - bounds.min_x) / (bounds.max_x - bounds.min_x) * canvas_width
    ));
}

[[nodiscard]] int svg_y(
    const MapBounds& bounds,
    double canvas_height,
    double y
) {
    return static_cast<int>(std::lround(
        canvas_height -
        (y - bounds.min_y) / (bounds.max_y - bounds.min_y) * canvas_height
    ));
}

void write_map_overlay_svg(
    const ScenarioRun& run,
    const std::filesystem::path& path
) {
    constexpr double canvas_width = 960.0;
    constexpr double canvas_height = 640.0;
    const MapBounds bounds = run.map.bounds();
    std::ofstream output(path, std::ios::trunc);
    ensure_stream(output, path);
    output << std::fixed << std::setprecision(3)
           << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 "
           << canvas_width << ' ' << canvas_height << "\">\n"
           << "<rect width=\"100%\" height=\"100%\" fill=\"#f4f1ea\"/>\n";

    for (int row = 0; row < run.map.height(); ++row) {
        for (int column = 0; column < run.map.width(); ++column) {
            const GridCell cell{row, column};
            if (!run.map.is_occupied(cell)) {
                continue;
            }
            const Vector2 centre = run.map.cell_center(cell);
            const double width = run.map.resolution_m() /
                (bounds.max_x - bounds.min_x) * canvas_width;
            const double height = run.map.resolution_m() /
                (bounds.max_y - bounds.min_y) * canvas_height;
            const double left = centre.x - run.map.resolution_m() / 2.0;
            const double top = centre.y + run.map.resolution_m() / 2.0;
            output << "<rect x=\"" << svg_x(bounds, canvas_width, left)
                   << "\" y=\"" << svg_y(bounds, canvas_height, top)
                   << "\" width=\"" << width << "\" height=\"" << height
                   << "\" fill=\"#293241\"/>\n";
        }
    }

    if (!run.planned_path.waypoints.empty()) {
        output << "<polyline fill=\"none\" stroke=\"#e07a5f\" stroke-width=\"5\" points=\"";
        for (const Vector2 point : run.planned_path.waypoints) {
            output << svg_x(bounds, canvas_width, point.x) << ','
                   << svg_y(bounds, canvas_height, point.y) << ' ';
        }
        output << "\"/>\n";
    }
    if (!run.simulation.trace.empty()) {
        output << "<polyline fill=\"none\" stroke=\"#2a9d8f\" stroke-width=\"3\" points=\"";
        for (const TraceSample& sample : run.simulation.trace) {
            output << svg_x(bounds, canvas_width, sample.pose.x) << ','
                   << svg_y(bounds, canvas_height, sample.pose.y) << ' ';
        }
        output << "\"/>\n";

        LidarSensor sensor(run.map, run.lidar_parameters);
        const Pose2D pose = run.simulation.trace.back().pose;
        const LaserScan scan = sensor.measure(pose);
        const std::size_t ray_stride = std::max<std::size_t>(
            1,
            scan.ranges_m.size() / 36
        );
        for (std::size_t index = 0; index < scan.ranges_m.size(); index += ray_stride) {
            const double world_angle = pose.theta + scan.angle_at(index);
            const double endpoint_x = pose.x + scan.ranges_m[index] * std::cos(world_angle);
            const double endpoint_y = pose.y + scan.ranges_m[index] * std::sin(world_angle);
            output << "<line x1=\"" << svg_x(bounds, canvas_width, pose.x)
                   << "\" y1=\"" << svg_y(bounds, canvas_height, pose.y)
                   << "\" x2=\"" << svg_x(bounds, canvas_width, endpoint_x)
                   << "\" y2=\"" << svg_y(bounds, canvas_height, endpoint_y)
                   << "\" stroke=\"#457b9d\" stroke-width=\"1\" opacity=\"0.45\"/>\n";
        }
    }

    const Vector2 start = run.map.cell_center(run.start_cell);
    const Vector2 goal = run.map.cell_center(run.goal_cell);
    output << "<circle cx=\"" << svg_x(bounds, canvas_width, start.x)
           << "\" cy=\"" << svg_y(bounds, canvas_height, start.y)
           << "\" r=\"10\" fill=\"#264653\"/>\n"
           << "<circle cx=\"" << svg_x(bounds, canvas_width, goal.x)
           << "\" cy=\"" << svg_y(bounds, canvas_height, goal.y)
           << "\" r=\"10\" fill=\"#e76f51\"/>\n";
    if (!run.simulation.trace.empty()) {
        const Pose2D pose = run.simulation.trace.back().pose;
        output << "<circle cx=\"" << svg_x(bounds, canvas_width, pose.x)
               << "\" cy=\"" << svg_y(bounds, canvas_height, pose.y)
               << "\" r=\"7\" fill=\"#f4a261\"/>\n"
               << "<line x1=\"" << svg_x(bounds, canvas_width, pose.x)
               << "\" y1=\"" << svg_y(bounds, canvas_height, pose.y)
               << "\" x2=\"" << svg_x(bounds, canvas_width, pose.x + 0.35 * std::cos(pose.theta))
               << "\" y2=\"" << svg_y(bounds, canvas_height, pose.y + 0.35 * std::sin(pose.theta))
               << "\" stroke=\"#f4a261\" stroke-width=\"4\"/>\n";
    }
    output << "<text x=\"20\" y=\"30\" font-family=\"sans-serif\" font-size=\"22\" fill=\"#293241\">"
           << simulation_status_name(run.simulation.status)
           << "</text>\n</svg>\n";
    ensure_stream(output, path);
}

[[nodiscard]] ScenarioRun run_scenario(const ExperimentConfig& config) {
    if (!std::isfinite(config.robot_radius_m) || config.robot_radius_m <= 0.0 ||
        !std::isfinite(config.simulation_dt_s) || config.simulation_dt_s <= 0.0 ||
        !std::isfinite(config.maximum_duration_s) || config.maximum_duration_s <= 0.0) {
        throw std::invalid_argument("experiment configuration contains invalid numeric values");
    }

    OccupancyGrid map = OccupancyGrid::load_from_file(config.map_path);
    const std::optional<GridCell> start_marker = map.start_marker();
    const std::optional<GridCell> goal_marker = map.goal_marker();
    if (!start_marker.has_value() || !goal_marker.has_value()) {
        throw std::invalid_argument(
            "experiment map must contain exactly one start marker and one goal marker"
        );
    }

    const AStarPlanner planner(map, config.robot_radius_m);
    const PlanResult plan = planner.plan_cells(*start_marker, *goal_marker);
    LidarParameters lidar_parameters;
    lidar_parameters.angle_min_rad = -0.5 * kPi;
    lidar_parameters.angle_max_rad = 0.5 * kPi;
    lidar_parameters.ray_count = 181;
    lidar_parameters.range_min_m = 0.02;
    lidar_parameters.range_max_m = 8.0;
    lidar_parameters.noise_stddev_m = 0.0;
    lidar_parameters.seed = config.random_seed;

    ScenarioRun run{
        std::move(map),
        *start_marker,
        *goal_marker,
        plan.path,
        plan.status,
        {},
        lidar_parameters,
    };
    if (!plan.success()) {
        run.simulation.status = SimulationStatus::InvalidInput;
        return run;
    }

    const Vector2 start = run.map.cell_center(run.start_cell);
    const Vector2 goal = run.map.cell_center(run.goal_cell);
    const PurePursuitController controller({
        0.75,
        0.20,
        0.40,
        0.50,
        2.0,
        2.0,
        1.5,
        0.10,
        0.12,
    });
    const DifferentialDriveKinematics kinematics({0.05, 0.30, 12.0});
    const CollisionChecker collision_checker(run.map, {config.robot_radius_m});
    LidarSensor lidar(run.map, run.lidar_parameters);
    const FixedStepSimulator simulator(
        controller,
        kinematics,
        collision_checker,
        {config.simulation_dt_s, config.maximum_duration_s},
        &lidar,
        nullptr
    );
    run.simulation = simulator.run(
        {start.x, start.y, 0.0},
        {goal.x, goal.y, 0.0},
        run.planned_path
    );
    return run;
}

}  // namespace

ExperimentSummary run_experiment(const ExperimentConfig& config) {
    ScenarioRun run = run_scenario(config);
    std::error_code error;
    std::filesystem::create_directories(config.output_directory, error);
    if (error) {
        throw std::runtime_error(
            "cannot create output directory " + config.output_directory.string() +
            ": " + error.message()
        );
    }

    write_resolved_config(
        config,
        run,
        config.output_directory / "resolved_config.yaml"
    );
    write_summary_json(config, run, config.output_directory / "summary.json");
    write_trace_csv(run.simulation, config.output_directory / "trace.csv");
    write_planned_path_csv(run.planned_path, config.output_directory / "planned_path.csv");
    write_lidar_csv(run, config.output_directory / "lidar.csv");
    write_map_overlay_svg(run, config.output_directory / "map_overlay.svg");

    return {
        run.plan_status,
        run.simulation.status,
        run.simulation.status == SimulationStatus::GoalReached,
        config.output_directory,
    };
}

}  // namespace robot_sim
