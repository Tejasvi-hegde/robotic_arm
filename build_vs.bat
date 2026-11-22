@echo off
REM Build RoboCell with Visual Studio 2022 + vcpkg (freeglut:x64-windows installed).
REM Change VCPKG_ROOT if vcpkg is not in C:\vcpkg.
set VCPKG_ROOT=C:\vcpkg
cd /d "%~dp0"
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows
if errorlevel 1 goto fail
cmake --build build --config Release
if errorlevel 1 goto fail
echo.
echo Running the algorithm tests...
build\Release\RoboCellTests.exe
echo.
echo Starting RoboCell...
start "" build\Release\RoboCell.exe
goto end
:fail
echo Build failed. Check that Visual Studio 2022, CMake and "vcpkg install freeglut:x64-windows" are installed.
pause
:end
