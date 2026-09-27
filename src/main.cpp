#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <filesystem>
#include <stdexcept>
#include <string_view>

#include "robot_sim/build_info.hpp"
#include "robot_sim/experiments/experiment_runner.hpp"

namespace {

void print_help(std::ostream& output) {
    output << "Usage:\n"
           << "  robot_sim --help\n"
           << "  robot_sim --version\n"
           << "  robot_sim --run MAP [--output DIRECTORY] [--seed N]\n\n"
           << "Runs a marker-based scenario and writes CSV, JSON, YAML, and SVG artifacts.\n"
           << "Default output directory: results/simple_room_goal\n";
}

[[nodiscard]] std::uint32_t parse_seed(std::string_view value) {
    std::size_t consumed = 0;
    const unsigned long parsed = std::stoul(std::string(value), &consumed);
    if (consumed != value.size() ||
        parsed > std::numeric_limits<std::uint32_t>::max()) {
        throw std::invalid_argument("seed must be an unsigned 32-bit integer");
    }
    return static_cast<std::uint32_t>(parsed);
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc == 1) {
        print_help(std::cout);
        return 0;
    }

    try {
        bool run_requested = false;
        std::filesystem::path map_path{"maps/simple_room.map"};
        std::filesystem::path output_directory{"results/simple_room_goal"};
        std::uint32_t seed = 42;

        for (int index = 1; index < argc; ++index) {
            const std::string_view argument(argv[index]);
            if (argument == "--help" || argument == "-h") {
                print_help(std::cout);
                return 0;
            }
            if (argument == "--version") {
                std::cout << robot_sim::kProjectName << " "
                          << robot_sim::kProjectVersion << '\n';
                return 0;
            }
            if (argument == "--run") {
                if (index + 1 >= argc) {
                    throw std::invalid_argument("--run requires a map path");
                }
                map_path = argv[++index];
                run_requested = true;
                continue;
            }
            if (argument == "--output") {
                if (index + 1 >= argc) {
                    throw std::invalid_argument("--output requires a directory");
                }
                output_directory = argv[++index];
                continue;
            }
            if (argument == "--seed") {
                if (index + 1 >= argc) {
                    throw std::invalid_argument("--seed requires a value");
                }
                seed = parse_seed(argv[++index]);
                continue;
            }

            throw std::invalid_argument(
                "unknown argument: " + std::string(argument)
            );
        }

        if (!run_requested) {
            print_help(std::cout);
            return 0;
        }

        const robot_sim::ExperimentSummary summary = robot_sim::run_experiment({
            map_path,
            output_directory,
            seed,
        });
        std::cout << "status: "
                  << robot_sim::simulation_status_name(summary.simulation_status)
                  << "\nartifacts: " << summary.output_directory.string() << '\n';
        return summary.success ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        print_help(std::cerr);
        return 2;
    }
}
