@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "BUILD_DIR=%SCRIPT_DIR%\build"
set "CMAKE_DIR=%SCRIPT_DIR%\cmake\bin"
set "BUILD_TYPE=Release"
set "EXECUTABLE=%BUILD_DIR%\hybrid_astar.exe"

if not exist "%CMAKE_DIR%\cmake.exe" (
    echo Error: CMake not found at %CMAKE_DIR%\cmake.exe
    echo Please download CMake from https://github.com/Kitware/CMake/releases
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

pushd "%SCRIPT_DIR%" >nul

if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo [run] Configuring CMake ^(%BUILD_TYPE%^)...
    "%CMAKE_DIR%\cmake.exe" -S "%SCRIPT_DIR%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
    if errorlevel 1 (
        popd >nul
        echo CMake configuration failed
        exit /b 1
    )
)

echo [run] Building hybrid_astar...
"%CMAKE_DIR%\cmake.exe" --build "%BUILD_DIR%" --config %BUILD_TYPE% --target hybrid_astar
if errorlevel 1 (
    popd >nul
    echo Build failed
    exit /b 1
)

if exist "%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar.exe" (
    set "EXECUTABLE=%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar.exe"
)

if not exist "%EXECUTABLE%" (
    popd >nul
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

if errorlevel 1 (
    popd >nul
    exit /b 1
)

echo.
echo [run] Opening output\result.json in path_json_viewer...
call tool\view_path_json.bat output\result.json
set "EXIT_CODE=%ERRORLEVEL%"
popd >nul
exit /b %EXIT_CODE%
