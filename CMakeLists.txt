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
  src/Robot/URDFRobot.cpp
  src/Renderer/STLLoader.cpp
  src/Renderer/FeatureEdges.cpp
  src/Trajectory/Trajectory.cpp
  src/Trajectory/Jog.h
  src/Trajectory/Jog.cpp
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

target_compile_definitions(ArmViz PRIVATE GLM_ENABLE_EXPERIMENTAL)

# INTERFACE_INCLUDE_DIRECTORIES can be a list; grab the first entry.
list(GET IMGUI_INCLUDE_DIR 0 IMGUI_INC0)



# Some builds also need the backend headers in include path (usually already covered)
target_include_directories(ArmViz PRIVATE
    "${IMGUI_INC0}/imgui/backends"
)

if (MSVC)
  target_compile_options(ArmViz PRIVATE /W4)
endif()


include(FetchContent)

FetchContent_Declare(
  tinyxml2
  GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
  GIT_TAG 10.0.0
)
FetchContent_MakeAvailable(tinyxml2)

target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2)
target_include_directories(${PROJECT_NAME} PRIVATE ${tinyxml2_SOURCE_DIR})

target_include_directories(ArmViz PRIVATE
    "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/include"
    "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/include/imgui"
    "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/include/imgui/backends"
)

target_compile_definitions(ArmViz PRIVATE
        PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)
