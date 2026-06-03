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
