@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "ROOT_DIR=%%~fI"
set "BUILD_DIR=%ROOT_DIR%\build"
set "CMAKE_DIR=%ROOT_DIR%\cmake\bin"
set "MAKE=mingw32-make.exe"
set "BUILD_TYPE=Release"
set "EXECUTABLE=%BUILD_DIR%\path_json_viewer.exe"
set "TARGET=output\result.json"
set "VIEWER_ARGS="
set "HAS_TARGET="

if /I "%~1"=="--help" goto :usage
if /I "%~1"=="-h" goto :usage

:parse_args
if "%~1"=="" goto :args_done
if not defined HAS_TARGET (
    set "FIRST=%~1"
    if not "!FIRST:~0,2!"=="--" (
        set "TARGET=%~1"
        set "HAS_TARGET=1"
        shift
        goto :parse_args
    )
)
set "VIEWER_ARGS=%VIEWER_ARGS% "%~1""
shift
goto :parse_args

:args_done
if not exist "%CMAKE_DIR%\cmake.exe" (
    echo Error: CMake not found at %CMAKE_DIR%\cmake.exe
    echo Please download CMake from https://github.com/Kitware/CMake/releases
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

pushd "%ROOT_DIR%" >nul

if not exist "%BUILD_DIR%\CMakeCache.txt" (
    echo [view-path] Configuring CMake ^(%BUILD_TYPE%^)...
    "%CMAKE_DIR%\cmake.exe" -B "%BUILD_DIR%" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=%BUILD_TYPE%
    if errorlevel 1 (
        popd >nul
        echo CMake configuration failed
        exit /b 1
    )
)

echo [view-path] Building path_json_viewer...
%MAKE% -C "%BUILD_DIR%" path_json_viewer
if errorlevel 1 (
    popd >nul
    echo Build failed
    exit /b 1
)

if not exist "%EXECUTABLE%" (
    popd >nul
    echo [view-path] Could not find path_json_viewer executable.
    exit /b 1
)

if exist "%TARGET%\*" (
    set "COUNT=0"
    for /f "delims=" %%F in ('dir /b /a:-d /o:n "%TARGET%\*.json" 2^>nul') do (
        set /a COUNT+=1
    )
    if "!COUNT!"=="0" (
        popd >nul
        echo [view-path] No *.json files found in directory: %TARGET%
        exit /b 1
    )

    echo [view-path] Opening !COUNT! JSON file^(s^) from %TARGET%...
    set "INDEX=0"
    for /f "delims=" %%F in ('dir /b /a:-d /o:n "%TARGET%\*.json" 2^>nul') do (
        set /a INDEX+=1
        echo [view-path] [!INDEX!/!COUNT!] %TARGET%\%%F
        call "%EXECUTABLE%" "%TARGET%\%%F"%VIEWER_ARGS%
        if errorlevel 1 (
            popd >nul
            exit /b 1
        )
    )
    popd >nul
    exit /b 0
)

if not exist "%TARGET%" (
    popd >nul
    echo [view-path] JSON file or directory not found: %TARGET%
    exit /b 1
)

echo [view-path] Opening %TARGET%...
call "%EXECUTABLE%" "%TARGET%"%VIEWER_ARGS%
set "EXIT_CODE=%ERRORLEVEL%"
popd >nul
exit /b %EXIT_CODE%

:usage
echo Usage: %~nx0 [result.json^|json_dir] [--path name^|index]
echo        %~nx0 [result.json^|json_dir] --list
echo        %~nx0 --list
echo        %~nx0 --help
echo.
echo If no JSON file or directory is provided, defaults to output\result.json.
echo If a directory is provided, opens all *.json files in that directory in sorted order.
exit /b 0
