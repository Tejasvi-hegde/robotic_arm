@echo off
REM Build RoboCell with MSYS2 MinGW (UCRT64):
REM   pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-freeglut
set PATH=C:\msys64\ucrt64\bin;%PATH%
cd /d "%~dp0"
set SRC=src\Simulation.cpp src\arm\Arm.cpp src\arm\IK.cpp src\arm\Behaviour.cpp src\physics\Collision.cpp src\physics\Force.cpp src\scene\Primitives.cpp src\scene\Sierpinski.cpp src\scene\Scene.cpp src\view\Camera.cpp src\view\Renderer.cpp src\input\Input.cpp src\raster\Raster2D.cpp src\raster\Clip2D.cpp src\ui\Hud.cpp
g++ -std=c++17 -O2 -Isrc src\main.cpp %SRC% -o RoboCell.exe -lfreeglut -lglu32 -lopengl32 -lwinmm -lgdi32
if errorlevel 1 goto fail
g++ -std=c++17 -O2 -Isrc tests\test_algorithms.cpp tests\test_globals.cpp %SRC% -o RoboCellTests.exe -lfreeglut -lglu32 -lopengl32 -lwinmm -lgdi32
RoboCellTests.exe
start "" RoboCell.exe
goto end
:fail
echo Build failed. Install MSYS2 and run: pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-freeglut
pause
:end
