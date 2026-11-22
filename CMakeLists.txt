cmake_minimum_required(VERSION 3.16)
project(RoboCell CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)        # freeGLUT (vcpkg: freeglut, Linux: freeglut3-dev)

set(CORE_SOURCES
    src/Simulation.cpp
    src/arm/Arm.cpp
    src/arm/IK.cpp
    src/arm/Behaviour.cpp
    src/physics/Collision.cpp
    src/physics/Force.cpp
    src/scene/Primitives.cpp
    src/scene/Sierpinski.cpp
    src/scene/Scene.cpp
    src/view/Camera.cpp
    src/view/Renderer.cpp
    src/input/Input.cpp
    src/raster/Raster2D.cpp
    src/raster/Clip2D.cpp
    src/ui/Hud.cpp
)

add_library(robocell_core STATIC ${CORE_SOURCES})
target_include_directories(robocell_core PUBLIC src ${GLUT_INCLUDE_DIRS})
target_link_libraries(robocell_core PUBLIC OpenGL::GL OpenGL::GLU GLUT::GLUT)

add_executable(RoboCell src/main.cpp)
target_link_libraries(RoboCell PRIVATE robocell_core)

# Algorithm tests (no window needed): FK/IK round trip, raster, clipping, collision
add_executable(RoboCellTests tests/test_algorithms.cpp tests/test_globals.cpp)
target_link_libraries(RoboCellTests PRIVATE robocell_core)

if(MSVC)
    target_compile_definitions(robocell_core PUBLIC _CRT_SECURE_NO_WARNINGS)
    target_compile_options(robocell_core PRIVATE /W3)
    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT RoboCell)
else()
    target_compile_options(robocell_core PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()
