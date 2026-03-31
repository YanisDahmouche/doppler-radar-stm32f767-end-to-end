@echo off
REM ═══════════════════════════════════════════════════════════════
REM  build.bat — Build radar_analyzer avec CMake + MinGW/MSVC
REM ═══════════════════════════════════════════════════════════════
REM  Usage (MSYS2 UCRT64 ou Developer Command Prompt) :
REM    cd chemin\vers\pc_analyzer
REM    build.bat
REM ═══════════════════════════════════════════════════════════════

echo.
echo ============================================
echo   Radar Analyzer — Build Script
echo ============================================
echo.

where cmake >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERREUR] CMake non trouve.
    pause
    exit /b 1
)

where git >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERREUR] Git non trouve.
    pause
    exit /b 1
)

if not exist build mkdir build
cd build

echo [1/2] Configuration CMake...
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
if %errorlevel% neq 0 (
    echo [INFO] MinGW non trouve, essai Ninja...
    cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
    if %errorlevel% neq 0 (
        echo [INFO] Essai VS 2022...
        cmake .. -G "Visual Studio 17 2022" -A x64
        if %errorlevel% neq 0 (
            echo [ERREUR] Configuration CMake echouee.
            cd ..
            pause
            exit /b 1
        )
    )
)

echo.
echo [2/2] Compilation...
cmake --build . --config Release --parallel
if %errorlevel% neq 0 (
    echo [ERREUR] Compilation echouee.
    cd ..
    pause
    exit /b 1
)

cd ..

echo.
echo ============================================
echo   BUILD OK !
echo   Executable : radar_analyzer.exe
echo   Usage : radar_analyzer.exe [fft_size]
echo   Exemple : radar_analyzer.exe 16384
echo ============================================
echo.
pause
