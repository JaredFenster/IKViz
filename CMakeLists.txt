cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE
    "C:/Users/jared/vcpkg/scripts/buildsystems/vcpkg.cmake"
    CACHE STRING "")

project(ArmViz LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(ArmViz
  src/main.cpp
  src/App.cpp
  src/Renderer/Shader.cpp
  src/Renderer/Mesh.cpp
  src/Renderer/Primitives.cpp
  src/Camera/OrbitCamera.cpp
  src/Robot/RobotScene.cpp
  src/Linker/origin.cpp
  src/Linker/linkJoint.cpp
  src/InverseKinematics/IK.cpp
)


target_link_libraries(ArmViz PRIVATE
    glfw
    glad::glad
    glm::glm
    imgui::imgui
    nlohmann_json::nlohmann_json
)

# ---------------- ImGui backends (GLFW + OpenGL3) ----------------
# vcpkg installs backend sources in the include directory.
get_target_property(IMGUI_INCLUDE_DIR imgui::imgui INTERFACE_INCLUDE_DIRECTORIES)

# INTERFACE_INCLUDE_DIRECTORIES can be a list; grab the first entry.
list(GET IMGUI_INCLUDE_DIR 0 IMGUI_INC0)



# Some builds also need the backend headers in include path (usually already covered)
target_include_directories(ArmViz PRIVATE
    "${IMGUI_INC0}/imgui/backends"
)

if (MSVC)
  target_compile_options(ArmViz PRIVATE /W4)
endif()
