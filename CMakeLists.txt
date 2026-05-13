cmake_minimum_required(VERSION 3.16)
project(gnss_ins_cpp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Eigen3 ────────────────────────────────────────────────────────
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# ── Main interface target (header-only) ───────────────────────────
add_library(gnss_ins INTERFACE)
target_include_directories(gnss_ins INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(gnss_ins INTERFACE Eigen3::Eigen)
target_compile_options(gnss_ins INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)

# ── Tests ─────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)

# ── CLI apps ──────────────────────────────────────────────────────
