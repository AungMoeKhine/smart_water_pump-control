@echo off
set "FILENAME=cloud_control.html"

:: Check if the file exists in the current directory
if exist "%~dp0%FILENAME%" (
    echo Launching Smart Pump App...
    
    :: Try to open with Chrome in App Mode (No URL bar)
    start chrome --app="file:///%~dp0%FILENAME%"
    
    :: If Chrome isn't found/default, fallback to default browser
    if %errorlevel% neq 0 (
        start "" "%~dp0%FILENAME%"
    )
) else (
    echo Error: %FILENAME% not found in this folder!
    pause
)
