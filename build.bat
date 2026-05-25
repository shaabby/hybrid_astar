@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "BUILD_DIR=%SCRIPT_DIR%build"
set "CMAKE_DIR=%SCRIPT_DIR%cmake\bin"
set "BUILD_TYPE=Release"

echo === Building Hybrid A* ===

if not exist "%CMAKE_DIR%\cmake.exe" (
    echo Error: CMake not found at %CMAKE_DIR%\cmake.exe
    echo Please download CMake from https://github.com/Kitware/CMake/releases
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo.
    echo [1/2] Running CMake...
    "%CMAKE_DIR%\cmake.exe" -S "%SCRIPT_DIR%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
    if errorlevel 1 (
        echo CMake configuration failed
        exit /b 1
    )
)

echo.
echo [2/2] Building...
"%CMAKE_DIR%\cmake.exe" --build "%BUILD_DIR%" --config %BUILD_TYPE%
if errorlevel 1 (
    echo Build failed
    exit /b 1
)

echo.
echo === Build complete ===
echo Executable: %BUILD_DIR%\hybrid_astar.exe
