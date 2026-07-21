# monitor/standalone/CMakeLists.txt
#
# A SEPARATE top-level CMake project that produces a solution containing ONLY
# the Vulkan renderer and the library dependencies it needs -- no CLI, no tests,
# no GoogleTest. Use it when you want to open Visual Studio straight onto the
# renderer:
#
#   cmake -S monitor/standalone -B build-renderer -G "Visual Studio 18 2026"
#   cmake --build build-renderer --target leo_monitor
#
# The main build (root CMakeLists.txt) is unchanged and still the canonical way
# to build everything; this is purely a convenience view. Nothing in src/,
# tests/, or the root targets is touched.
#
# It reuses the existing monitor/CMakeLists.txt as the single source of truth
# for the renderer target. The only thing duplicated here is the definition of
# the `constellation` + `sgp4` libraries the renderer links -- kept in sync with
# the root by mirroring the same source list.

cmake_minimum_required(VERSION 3.20)
project(leo_renderer CXX)  # -> leo_renderer.slnx

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Repo root, two directories up from this file.
get_filename_component(LEO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

include(FetchContent)
FetchContent_Declare(unordered_dense
  GIT_REPOSITORY https://github.com/martinus/unordered_dense.git
  GIT_TAG v4.4.0)
FetchContent_Declare(glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 1.0.1)
FetchContent_Declare(nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3)
FetchContent_MakeAvailable(unordered_dense glm nlohmann_json)

# --- sgp4 (mirror of the root definition) -----------------------------------
add_library(sgp4 STATIC ${LEO_ROOT}/third_party/sgp4/SGP4.cpp)
target_include_directories(sgp4 PUBLIC ${LEO_ROOT}/third_party/sgp4)
if(MSVC)
  target_compile_definitions(sgp4 PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# --- constellation library (mirror of the root definition, same sources) ----
add_library(constellation
  ${LEO_ROOT}/src/constellation.cpp
  ${LEO_ROOT}/src/omm_loader.cpp
  ${LEO_ROOT}/src/topology.cpp
  ${LEO_ROOT}/src/epoch_health.cpp
  ${LEO_ROOT}/src/propagator.cpp
  ${LEO_ROOT}/src/isl_links.cpp
  ${LEO_ROOT}/src/ground.cpp
  ${LEO_ROOT}/src/router.cpp
  ${LEO_ROOT}/src/ground_stations.cpp
  ${LEO_ROOT}/src/simulator.cpp
  ${LEO_ROOT}/src/cli.cpp
)
target_include_directories(constellation PUBLIC ${LEO_ROOT}/src)
target_compile_features(constellation PUBLIC cxx_std_20)
target_link_libraries(constellation PUBLIC
  unordered_dense::unordered_dense
  glm::glm)
target_link_libraries(constellation PRIVATE
  nlohmann_json::nlohmann_json
  sgp4)

# --- the renderer: reuse the real monitor/CMakeLists.txt --------------------
# It defines find_package(Vulkan), the GLFW fetch, leo_monitor, the shader
# target, etc. -- and links the `constellation` target defined above. Give it an
# explicit binary dir since it lives outside this source tree.
add_subdirectory(${LEO_ROOT}/monitor ${CMAKE_BINARY_DIR}/monitor)
