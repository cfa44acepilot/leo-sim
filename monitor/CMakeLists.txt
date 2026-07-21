# monitor/CMakeLists.txt -- the optional Vulkan renderer.
#
# This file is included ONLY when LEO_BUILD_MONITOR=ON (the root CMakeLists
# guards the add_subdirectory). So find_package(Vulkan) and the GLFW fetch run
# only when the renderer is explicitly requested -- a default build, or a Linux
# CI that just wants the simulator + CLI, never touches Vulkan or GLFW.
#
# Portability rules honored here:
#   * No hardcoded compiler and no absolute SDK paths. Vulkan is located via
#     find_package(Vulkan), which reads the VULKAN_SDK env var the SDK sets on
#     both Windows and Linux.
#   * GLFW is built from source via FetchContent so no system package is needed.
#   * Warning flags are chosen by compiler-driver variant so the Windows
#     clang-cl (MSVC driver) build does not choke on GNU-style flags.

find_package(Vulkan REQUIRED)

include(FetchContent)
# Build GLFW from source: identical on Windows and Linux, no preinstall needed.
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

# Dear ImGui: not a CMake project, so we fetch the sources and build our own
# static lib from the core + the GLFW and Vulkan backends. Pinned to v1.90.9,
# whose Vulkan backend API (UseDynamicRendering + ColorAttachmentFormat) the
# renderer code matches exactly.
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.90.9
)
FetchContent_MakeAvailable(imgui)
add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)
target_include_directories(imgui PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw Vulkan::Vulkan)

# stb_image: single-header JPEG/PNG/etc decoder (public domain). Used to load
# the Earth day-map .jpg. (It does NOT decode TIFF -- normal/specular maps would
# need converting to PNG first.) The repo has no CMakeLists, so we just expose
# its include dir; STB_IMAGE_IMPLEMENTATION is compiled once in stb_impl.cpp.
FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG master
)
FetchContent_MakeAvailable(stb)

add_executable(leo_monitor
  src/main.cpp
  src/vk_renderer.cpp
  src/scene.cpp
  src/stb_impl.cpp
  src/data_update.cpp
  src/route_diag.cpp
)
target_include_directories(leo_monitor PRIVATE ${stb_SOURCE_DIR})

# Link the SAME simulator library the CLI links, read-only. Vulkan + GLFW + ImGui
# are private implementation details of this executable.
target_link_libraries(leo_monitor PRIVATE
  constellation
  Vulkan::Vulkan
  glfw
  imgui
)

# Match the core's C++ standard (the constellation lib requires C++20).
target_compile_features(leo_monitor PRIVATE cxx_std_20)

# Vulkan clip-space conventions for GLM: depth 0..1 and radians.
target_compile_definitions(leo_monitor PRIVATE
  GLM_FORCE_RADIANS
  GLM_FORCE_DEPTH_ZERO_TO_ONE
)

# Warnings: clang-cl uses the MSVC driver (/W3), clang++ the GNU driver (-Wall).
# Selecting by frontend variant keeps configure clean under BOTH drivers.
if(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
  target_compile_options(leo_monitor PRIVATE /W3)
else()
  target_compile_options(leo_monitor PRIVATE -Wall -Wextra)
endif()

# When launched from Visual Studio (F5), run from the repo root and with sensible
# default endpoints, so the relative `data/*.json` paths resolve. monitor/ is
# always <repo>/monitor, so its parent is the repo root in every build tree.
get_filename_component(LEO_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set_target_properties(leo_monitor PROPERTIES
  VS_DEBUGGER_WORKING_DIRECTORY "${LEO_REPO_ROOT}"
  VS_DEBUGGER_COMMAND_ARGUMENTS "Chicago London")

# --- Shader compilation: GLSL -> SPIR-V at build time -----------------------
# Use the compiler shipped in the Vulkan SDK (found via find_package(Vulkan),
# never a literal path). Prefer glslc; fall back to glslangValidator.
if(Vulkan_GLSLC_EXECUTABLE)
  set(LEO_GLSL_COMPILER "${Vulkan_GLSLC_EXECUTABLE}")
  set(LEO_GLSL_DRIVER "glslc")
elseif(Vulkan_GLSLANG_VALIDATOR_EXECUTABLE)
  set(LEO_GLSL_COMPILER "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}")
  set(LEO_GLSL_DRIVER "glslang")
else()
  message(FATAL_ERROR
    "No GLSL->SPIR-V compiler (glslc/glslangValidator) found in the Vulkan SDK")
endif()

set(LEO_SPV_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY "${LEO_SPV_DIR}")

set(LEO_SHADERS
  mesh.vert mesh.frag point.vert point.frag line.vert line.frag
  earth.vert earth.frag route.vert route.frag select.vert select.frag
  marker.frag)

set(LEO_SPV_FILES)
foreach(shader ${LEO_SHADERS})
  set(src "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${shader}")
  set(spv "${LEO_SPV_DIR}/${shader}.spv")
  if(LEO_GLSL_DRIVER STREQUAL "glslc")
    add_custom_command(
      OUTPUT "${spv}"
      COMMAND "${LEO_GLSL_COMPILER}" "${src}" -o "${spv}"
      DEPENDS "${src}"
      COMMENT "glslc ${shader}"
      VERBATIM)
  else()
    add_custom_command(
      OUTPUT "${spv}"
      COMMAND "${LEO_GLSL_COMPILER}" -V "${src}" -o "${spv}"
      DEPENDS "${src}"
      COMMENT "glslangValidator ${shader}"
      VERBATIM)
  endif()
  list(APPEND LEO_SPV_FILES "${spv}")
endforeach()

add_custom_target(leo_monitor_shaders DEPENDS ${LEO_SPV_FILES})
add_dependencies(leo_monitor leo_monitor_shaders)

# Tell the renderer where the compiled .spv files live (forward slashes work on
# Windows std::ifstream too), and where the source texture assets are.
target_compile_definitions(leo_monitor PRIVATE
  MONITOR_SHADER_DIR="${LEO_SPV_DIR}"
  MONITOR_TEXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/textures")
