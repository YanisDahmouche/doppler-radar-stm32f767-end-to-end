@echo off
REM ═══════════════════════════════════════════════════════════════
REM  build.bat — Build radar_scope avec CMake + MSVC
REM ═══════════════════════════════════════════════════════════════
REM  Prérequis :
REM    1. Visual Studio 2019/2022 avec "Desktop C++ workload"
REM       OU Build Tools for Visual Studio (gratuit)
REM    2. CMake 3.20+ (inclus avec VS, ou cmake.org)
REM    3. Git (pour FetchContent, télécharge GLFW + ImGui)
REM
REM  Utilisation :
REM    Ouvrir "Developer Command Prompt for VS" puis :
REM      cd chemin\vers\pc_scope
REM      build.bat
REM
REM  Résultat : radar_scope.exe dans ce dossier
REM ═══════════════════════════════════════════════════════════════

echo.
echo ============================================
echo   Radar Scope — Build Script
echo ============================================
echo.

REM Vérifier CMake
where cmake >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERREUR] CMake non trouve dans le PATH.
    echo          Ouvrez "Developer Command Prompt for VS" ou installez CMake.
    pause
    exit /b 1
)

REM Vérifier Git
where git >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERREUR] Git non trouve dans le PATH.
    echo          Installez Git depuis https://git-scm.com
    pause
    exit /b 1
)

REM Créer le dossier de build
if not exist build mkdir build
cd build

echo [1/2] Configuration CMake...
cmake .. -G "Visual Studio 17 2022" -A x64
if %errorlevel% neq 0 (
    echo.
    echo [INFO] VS 2022 non trouve, essai avec VS 2019...
    cmake .. -G "Visual Studio 16 2019" -A x64
    if %errorlevel% neq 0 (
        echo.
        echo [INFO] Essai avec Ninja...
        cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
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
echo   Executable : radar_scope.exe
echo   Usage : radar_scope.exe [COMx]
echo ============================================
echo.
pause
