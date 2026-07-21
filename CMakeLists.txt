cmake_minimum_required(VERSION 3.20)
project(leo_constellation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# Header-only fast hashmap for the cold catalog->Satellite map.
FetchContent_Declare(
  unordered_dense
  GIT_REPOSITORY https://github.com/martinus/unordered_dense.git
  GIT_TAG v4.4.0
)

# Vector math; we use glm::dvec3 (double precision) for all state vectors.
FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 1.0.1
)

# Header-only JSON parser for the OMM snapshot loader. Hand-rolled parsing is
# banned, so the loader depends on this.
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.2
)
# Match the MSVC runtime gtest links against to ours (avoids LNK mismatch).
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(unordered_dense glm nlohmann_json googletest)

# Vendored Vallado SGP4 (public domain). Compiled as its own static lib so its
# warnings/flags stay isolated; only propagator.cpp consumes its header.
add_library(sgp4 STATIC third_party/sgp4/SGP4.cpp)
target_include_directories(sgp4 PUBLIC third_party/sgp4)
# Reference C code uses sprintf/strcpy; silence MSVC's "unsafe" deprecations
# rather than patching vendored source.
if(MSVC)
  target_compile_definitions(sgp4 PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

add_library(constellation
  src/constellation.cpp
  src/omm_loader.cpp
  src/topology.cpp
  src/epoch_health.cpp
  src/propagator.cpp
  src/isl_links.cpp
  src/ground.cpp
  src/router.cpp
  src/ground_stations.cpp
  src/simulator.cpp
  src/cli.cpp
)
target_include_directories(constellation PUBLIC src)
target_compile_features(constellation PUBLIC cxx_std_20)
target_link_libraries(constellation PUBLIC
  unordered_dense::unordered_dense
  glm::glm
)
# nlohmann/json and SGP4 are implementation details (used only in .cpp files),
# so keep them off the library's public interface.
target_link_libraries(constellation PRIVATE
  nlohmann_json::nlohmann_json
  sgp4
)

# The `leo` command-line front end over the Simulator library.
add_executable(leo src/main.cpp)
target_link_libraries(leo PRIVATE constellation)

enable_testing()

add_executable(constellation_test tests/constellation_test.cpp)
target_link_libraries(constellation_test PRIVATE
  constellation
  GTest::gtest_main
)

add_executable(omm_loader_test tests/omm_loader_test.cpp)
target_link_libraries(omm_loader_test PRIVATE
  constellation
  GTest::gtest_main
)
# Hand the test the absolute path to the committed fixtures so it is independent
# of the working directory and the build location.
target_compile_definitions(omm_loader_test PRIVATE
  LEO_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

add_executable(topology_test tests/topology_test.cpp)
target_link_libraries(topology_test PRIVATE
  constellation
  GTest::gtest_main
)

add_executable(epoch_health_test tests/epoch_health_test.cpp)
target_link_libraries(epoch_health_test PRIVATE
  constellation
  GTest::gtest_main
)

add_executable(propagator_test tests/propagator_test.cpp)
target_link_libraries(propagator_test PRIVATE
  constellation
  GTest::gtest_main
)

add_executable(isl_links_test tests/isl_links_test.cpp)
target_link_libraries(isl_links_test PRIVATE
  constellation
  GTest::gtest_main
)

add_executable(ground_test tests/ground_test.cpp)
target_link_libraries(ground_test PRIVATE
  constellation
  GTest::gtest_main
)

add_executable(router_test tests/router_test.cpp)
target_link_libraries(router_test PRIVATE
  constellation
  GTest::gtest_main
)

add_executable(ground_stations_test tests/ground_stations_test.cpp)
target_link_libraries(ground_stations_test PRIVATE
  constellation
  GTest::gtest_main
)
target_compile_definitions(ground_stations_test PRIVATE
  LEO_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

add_executable(simulator_test tests/simulator_test.cpp)
target_link_libraries(simulator_test PRIVATE
  constellation
  GTest::gtest_main
)
target_compile_definitions(simulator_test PRIVATE
  LEO_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

add_executable(cli_test tests/cli_test.cpp)
target_link_libraries(cli_test PRIVATE
  constellation
  GTest::gtest_main
)
target_compile_definitions(cli_test PRIVATE
  LEO_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

include(GoogleTest)
gtest_discover_tests(constellation_test)
gtest_discover_tests(omm_loader_test)
gtest_discover_tests(topology_test)
gtest_discover_tests(epoch_health_test)
gtest_discover_tests(propagator_test)
gtest_discover_tests(isl_links_test)
gtest_discover_tests(ground_test)
gtest_discover_tests(router_test)
gtest_discover_tests(ground_stations_test)
gtest_discover_tests(simulator_test)
gtest_discover_tests(cli_test)

# --- Optional Vulkan renderer (isolated frontend) ---------------------------
# OFF by default: a plain `cmake` build produces exactly the simulator + CLI +
# tests above, with NO Vulkan/GLFW dependency. The renderer, and every package
# it needs, are confined to monitor/ and only enter the build graph when this
# option is ON. Nothing above is modified by it.
option(LEO_BUILD_MONITOR "Build the optional Vulkan renderer (monitor/)" OFF)
if(LEO_BUILD_MONITOR)
  add_subdirectory(monitor)
endif()
