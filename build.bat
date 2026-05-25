@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "BUILD_DIR=%SCRIPT_DIR%build"
set "CMAKE_DIR=%SCRIPT_DIR%cmake\bin"
set "MAKE=mingw32-make.exe"

echo === Building Hybrid A* ===

if not exist "%CMAKE_DIR%\cmake.exe" (
    echo Error: CMake not found at %CMAKE_DIR%\cmake.exe
    echo Please download CMake from https://github.com/Kitware/CMake/releases
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

echo.
echo [1/2] Running CMake...
"%CMAKE_DIR%\cmake.exe" -B "%BUILD_DIR%" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 (
    echo CMake configuration failed
    exit /b 1
)

echo.
echo [2/2] Building...
"%MAKE%" -C "%BUILD_DIR%"
if errorlevel 1 (
    echo Build failed
    exit /b 1
)

echo.
echo === Build complete ===
echo Executable: %BUILD_DIR%\hybrid_astar.exe