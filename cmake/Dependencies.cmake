include(FetchContent)

set(ROBOT_SIM_EIGEN_AVAILABLE OFF)
set(ROBOT_SIM_GTEST_AVAILABLE OFF)
set(ROBOT_SIM_RAYLIB_AVAILABLE OFF)
set(ROBOT_SIM_RAYLIB_TARGET "")

if(ROBOT_SIM_BUILD_VIEWER)
    find_package(raylib QUIET CONFIG)
    if(TARGET raylib)
        set(ROBOT_SIM_RAYLIB_AVAILABLE ON)
        set(ROBOT_SIM_RAYLIB_TARGET raylib)
    elseif(TARGET raylib::raylib)
        set(ROBOT_SIM_RAYLIB_AVAILABLE ON)
        set(ROBOT_SIM_RAYLIB_TARGET raylib::raylib)
    endif()
endif()

if(ROBOT_SIM_FETCH_DEPENDENCIES)
    set(BUILD_TESTING OFF CACHE BOOL "Disable dependency tests" FORCE)

    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
    )
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

    if(ROBOT_SIM_FETCH_RAYLIB)
        set(BUILD_EXAMPLES OFF CACHE BOOL "Disable raylib examples" FORCE)
        set(BUILD_GAMES OFF CACHE BOOL "Disable raylib games" FORCE)
        FetchContent_Declare(
            raylib
            URL https://github.com/raysan5/raylib/archive/refs/tags/6.0.tar.gz
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )
    endif()

    if(ROBOT_SIM_FETCH_RAYLIB)
        FetchContent_MakeAvailable(eigen googletest raylib)
        if(TARGET raylib)
            set(ROBOT_SIM_RAYLIB_AVAILABLE ON)
            set(ROBOT_SIM_RAYLIB_TARGET raylib)
        elseif(TARGET raylib::raylib)
            set(ROBOT_SIM_RAYLIB_AVAILABLE ON)
            set(ROBOT_SIM_RAYLIB_TARGET raylib::raylib)
        endif()
    else()
        FetchContent_MakeAvailable(eigen googletest)
    endif()
    set(ROBOT_SIM_EIGEN_AVAILABLE ON)
    set(ROBOT_SIM_GTEST_AVAILABLE ON)
else()
    find_package(Eigen3 3.3 QUIET NO_MODULE)
    if(TARGET Eigen3::Eigen)
        set(ROBOT_SIM_EIGEN_AVAILABLE ON)
    else()
        message(STATUS "Eigen3 not found; Phase 1 uses project-local small geometry types")
    endif()

    find_package(GTest QUIET)
    if(TARGET GTest::gtest_main)
        set(ROBOT_SIM_GTEST_AVAILABLE ON)
    else()
        message(STATUS "GoogleTest not found; using the standard-library unit-test fallback")
    endif()
endif()

unset(BUILD_TESTING CACHE)
