@echo off
setlocal

set "ROOT=%~dp0"
set "ROOT=%ROOT:~0,-1%"
set "BUILD_DIR=%ROOT%\build"
set "BUILD_TYPE=Release"
set "PLANNER=%BUILD_DIR%\hybrid_astar.exe"
set "VIEWER=%BUILD_DIR%\path_json_viewer.exe"

:: --- check cmake ---
where cmake >nul 2>&1 || (
    echo Error: CMake not found in PATH
    exit /b 1
)

cd /d "%ROOT%"

:: --- build ---
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo [run] Configuring CMake (%BUILD_TYPE%)...
    cmake -S "%ROOT%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
)
echo [run] Building...
cmake --build "%BUILD_DIR%" --config %BUILD_TYPE%

:: --- locate planner ---
if exist "%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar.exe" (
    set "PLANNER=%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar.exe"
)
if exist "%BUILD_DIR%\%BUILD_TYPE%\path_json_viewer.exe" (
    set "VIEWER=%BUILD_DIR%\%BUILD_TYPE%\path_json_viewer.exe"
)

if not exist "%PLANNER%" (
    echo Error: planner not found: %PLANNER%
    exit /b 1
)

:: --- run planner ---
if "%~1"=="" (
    echo [run] Running config/default.yaml...
    "%PLANNER%" config\default.yaml
) else (
    echo [run] Running %*...
    "%PLANNER%" %*
)

:: --- open viewer ---
if exist "output\result.json" (
    echo.
    echo [run] Opening output\result.json...
    "%VIEWER%" output\result.json
)
