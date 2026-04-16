cmake_minimum_required(VERSION 3.10)
project(MyProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== vcpkg  =====
find_package(imgui CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenGL REQUIRED)
# GLFW3 can also be used directly from vcpkg.
find_package(glfw3 CONFIG REQUIRED)

# ===== include  =====
include_directories(include)
include_directories(/usr/include/OpenEXR)
include_directories(/usr/include/Imath)

# ===== Gurobi =====
set(GUROBI_HOME $ENV{GUROBI_HOME})
include_directories(${GUROBI_HOME}/include)
link_directories(${GUROBI_HOME}/lib)

# ==========
file(GLOB_RECURSE SOURCES src/*.cpp)

add_executable(main ${SOURCES})

# ===== Link  =====
target_link_libraries(main
    PRIVATE
    imgui::imgui
    glfw
    yaml-cpp
    OpenGL::GL
    Imath
    gurobi_c++
    gurobi130
    m
    pthread
    dl
)
