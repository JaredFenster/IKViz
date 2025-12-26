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

add_executable(ArmViz src/main.cpp)

target_link_libraries(ArmViz PRIVATE
    glfw
    glad::glad
    glm::glm
    imgui::imgui
)

if (MSVC)
  target_compile_options(ArmViz PRIVATE /W4)
endif()
