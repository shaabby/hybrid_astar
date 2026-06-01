@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "BUILD_DIR=%SCRIPT_DIR%\build"
set "CMAKE_DIR=%SCRIPT_DIR%\cmake\bin"
set "BUILD_TYPE=Release"
set "TESTBENCH=%BUILD_DIR%\hybrid_astar_testbench.exe"
set "OUTPUT_CSV=output\demo.csv"
set "OUTPUT_MAP_DIR=output\demo"
set "VIEW_DIR=%OUTPUT_MAP_DIR%\default"

if not exist "%CMAKE_DIR%\cmake.exe" (
    echo Error: CMake not found at %CMAKE_DIR%\cmake.exe
    echo Please download CMake from https://github.com/Kitware/CMake/releases
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

pushd "%SCRIPT_DIR%" >nul

if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo [demo] Configuring CMake ^(%BUILD_TYPE%^)...
    "%CMAKE_DIR%\cmake.exe" -S "%SCRIPT_DIR%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
    if errorlevel 1 (
        popd >nul
        echo CMake configuration failed
        exit /b 1
    )
)

echo [demo] Building hybrid_astar_testbench...
"%CMAKE_DIR%\cmake.exe" --build "%BUILD_DIR%" --config %BUILD_TYPE% --target hybrid_astar_testbench
if errorlevel 1 (
    popd >nul
    echo Build failed
    exit /b 1
)

if exist "%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar_testbench.exe" (
    set "TESTBENCH=%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar_testbench.exe"
)

if not exist "%TESTBENCH%" (
    popd >nul
    echo Error: Executable not found: %TESTBENCH%
    exit /b 1
)

if exist "%OUTPUT_CSV%" del /q "%OUTPUT_CSV%"
if exist "%OUTPUT_MAP_DIR%" rmdir /s /q "%OUTPUT_MAP_DIR%"

echo [demo] Running default config on maps in map\ ...
"%TESTBENCH%" ^
  --groups config/testbench/default_groups.txt ^
  --maps map ^
  --output "%OUTPUT_CSV%" ^
  --output-map-dir "%OUTPUT_MAP_DIR%"
if errorlevel 1 (
    popd >nul
    exit /b 1
)

echo [demo] Opening all generated path JSON files...
call tool\view_path_json.bat "%VIEW_DIR%"
set "EXIT_CODE=%ERRORLEVEL%"

popd >nul
exit /b %EXIT_CODE%
