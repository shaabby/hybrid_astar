@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "ROOT_DIR=%SCRIPT_DIR:~0,-1%"
set "BUILD_DIR=%ROOT_DIR%\build"
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
if not exist "%EXECUTABLE%" (
    if exist "%BUILD_DIR%\%BUILD_TYPE%\path_json_viewer.exe" (
        echo Found existing executable from build: "%BUILD_DIR%\%BUILD_TYPE%\path_json_viewer.exe"
        set "EXECUTABLE=%BUILD_DIR%\%BUILD_TYPE%\path_json_viewer.exe"
    )
)

if not exist "%EXECUTABLE%" (
    echo "%EXECUTABLE%" not found.
    echo [view-path] Could not find path_json_viewer executable.
    echo Please build first using build.bat
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
