@echo off

if exist "%~dp0\..\VisualizerApp\VisualizerApp.exe" (
    "%~dp0\..\VisualizerApp\VisualizerApp.exe" "%~dp0data"
) else (
    echo VisualizerApp must have been packaged with the 'package.bat' script prior testing it.
)
