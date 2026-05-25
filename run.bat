@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "BUILD_DIR=%SCRIPT_DIR%build"
set "CMAKE_DIR=%SCRIPT_DIR%cmake\bin"
set "MAKE=mingw32-make.exe"
set "BUILD_TYPE=Release"
set "EXECUTABLE=%BUILD_DIR%\hybrid_astar.exe"

if not exist "%CMAKE_DIR%\cmake.exe" (
    echo Error: CMake not found at %CMAKE_DIR%\cmake.exe
    echo Please download CMake from https://github.com/Kitware/CMake/releases
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo [run] Configuring CMake ^(%BUILD_TYPE%^)...
    "%CMAKE_DIR%\cmake.exe" -B "%BUILD_DIR%" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
    if errorlevel 1 (
        echo CMake configuration failed
        exit /b 1
    )
)

echo [run] Building hybrid_astar...
%MAKE% -C "%BUILD_DIR%"
if errorlevel 1 (
    echo Build failed
    exit /b 1
)

if not exist "%EXECUTABLE%" (
    echo Error: Executable not found: %EXECUTABLE%
    exit /b 1
)

if "%~1"=="" (
    echo [run] Running config/default.yaml...
    "%EXECUTABLE%" config/default.yaml
) else (
    echo [run] Running %*
    "%EXECUTABLE%" %*
)

if errorlevel 1 exit /b 1

echo.
echo [run] Done.
