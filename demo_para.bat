@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "BUILD_DIR=%SCRIPT_DIR%\build"
set "BUILD_TYPE=Release"
set "TESTBENCH=%BUILD_DIR%\hybrid_astar_testbench.exe"
set "GROUPS_FILE=config/testbench/demo_para/groups.txt"
set "MAP_SOURCE=map\empty01.json"
set "MAP_DIR=output\demo_para\maps"
set "OUTPUT_CSV=output\demo_para.csv"
set "OUTPUT_MAP_DIR=output\demo_para"

if not exist "%TESTBENCH%" (
    if exist "%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar_testbench.exe" (
        set "TESTBENCH=%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar_testbench.exe"
    )
)

if not exist "%TESTBENCH%" (
    echo Error: Executable not found: %TESTBENCH%
    echo Please build first using build.bat
    exit /b 1
)

pushd "%SCRIPT_DIR%" >nul

if not exist "%MAP_SOURCE%" (
    popd >nul
    echo Error: Map not found: %MAP_SOURCE%
    exit /b 1
)

if exist "%OUTPUT_CSV%" del /q "%OUTPUT_CSV%"
if exist "%OUTPUT_MAP_DIR%" rmdir /s /q "%OUTPUT_MAP_DIR%"

mkdir "%MAP_DIR%"
copy /y "%MAP_SOURCE%" "%MAP_DIR%\empty01.json" >nul
if errorlevel 1 (
    popd >nul
    echo Failed to prepare map directory
    exit /b 1
)

echo [demo-para] Running three parameter groups on map\empty01.json ...
"%TESTBENCH%" ^
  --groups "%GROUPS_FILE%" ^
  --maps "%MAP_DIR%" ^
  --output "%OUTPUT_CSV%" ^
  --output-map-dir "%OUTPUT_MAP_DIR%"
if errorlevel 1 (
    popd >nul
    exit /b 1
)

echo [demo-para] Opening generated path JSON files...
call tool\view_path_json.bat "%OUTPUT_MAP_DIR%\fine"
if errorlevel 1 goto :viewer_failed
call tool\view_path_json.bat "%OUTPUT_MAP_DIR%\medium"
if errorlevel 1 goto :viewer_failed
call tool\view_path_json.bat "%OUTPUT_MAP_DIR%\coarse"
if errorlevel 1 goto :viewer_failed
set "EXIT_CODE=%ERRORLEVEL%"
goto :done

:viewer_failed
set "EXIT_CODE=%ERRORLEVEL%"

:done
popd >nul
exit /b %EXIT_CODE%
