#ifndef ROBOT_SIM_HAS_GTEST

void run_kinematics_tests();
void run_map_and_collision_tests();
void run_planning_tests();
void run_experiment_runner_tests();
void run_hardening_tests();
void run_sensor_and_safety_tests();
void run_simulation_tests();
void run_smoke_tests();

int main() {
    run_smoke_tests();
    run_kinematics_tests();
    run_map_and_collision_tests();
    run_planning_tests();
    run_experiment_runner_tests();
    run_hardening_tests();
    run_sensor_and_safety_tests();
    run_simulation_tests();
    return 0;
}

#endif
