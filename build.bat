@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "BUILD_DIR=%SCRIPT_DIR%\build"
set "BUILD_TYPE=Release"

echo === Building all targets ===

where cmake >nul 2>&1
if errorlevel 1 (
    echo Error: cmake not found in PATH
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

pushd "%SCRIPT_DIR%" >nul

if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo.
    echo [1/2] Running CMake...
    cmake -S "%SCRIPT_DIR%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
    if errorlevel 1 (
        popd >nul
        echo CMake configuration failed
        exit /b 1
    )
)

echo.
echo [2/2] Building...
cmake --build "%BUILD_DIR%" --config %BUILD_TYPE%
if errorlevel 1 (
    popd >nul
    echo Build failed
    exit /b 1
)

popd >nul

echo.
echo === Build complete ===
echo Executables in: %BUILD_DIR%\%BUILD_TYPE%\
