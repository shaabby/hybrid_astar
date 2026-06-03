@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "BUILD_DIR=%SCRIPT_DIR%\build"
set "BUILD_TYPE=Release"
set "EXECUTABLE=%BUILD_DIR%\hybrid_astar.exe"

if exist "%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar.exe" (
    set "EXECUTABLE=%BUILD_DIR%\%BUILD_TYPE%\hybrid_astar.exe"
)

if not exist "%EXECUTABLE%" (
    echo Error: Executable not found: %EXECUTABLE%
    echo Please build first using build.bat
    exit /b 1
)

pushd "%SCRIPT_DIR%" >nul

if "%~1"=="" (
    echo [run] Running config/default.yaml...
    "%EXECUTABLE%" config/default.yaml
) else (
    echo [run] Running %*...
    "%EXECUTABLE%" %*
)

echo.
echo [run] Opening output/result.json in path_json_viewer...
call tool\view_path_json.bat output\result.json

popd >nul
