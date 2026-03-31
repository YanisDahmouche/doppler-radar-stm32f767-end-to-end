cmake_minimum_required(VERSION 3.20)
project(radar_scope LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ═══════════════════════════════════════════════════════════════
# FetchContent : télécharge automatiquement GLFW et Dear ImGui
# ═══════════════════════════════════════════════════════════════
include(FetchContent)

# ── GLFW 3.4 ──
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# ── Dear ImGui v1.91.8 (docking branch) ──
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# Construire la bibliothèque ImGui à partir des sources
set(IMGUI_DIR ${imgui_SOURCE_DIR})
add_library(imgui_lib STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw)

# ═══════════════════════════════════════════════════════════════
# Application principale
# ═══════════════════════════════════════════════════════════════
add_executable(radar_scope WIN32 radar_scope.cpp)

target_link_libraries(radar_scope PRIVATE
    imgui_lib
    glfw
    opengl32      # Win32 OpenGL
    gdi32         # MinGW needs these explicitly
    imm32
    dwmapi
)

# Définitions utiles
target_compile_definitions(radar_scope PRIVATE
    _CRT_SECURE_NO_WARNINGS
    _USE_MATH_DEFINES
)

# Optimisations Release
if(MSVC)
    target_compile_options(radar_scope PRIVATE /O2 /W3)
else()
    target_compile_options(radar_scope PRIVATE -O2 -Wall)
endif()

# Copier l'exécutable dans le dossier racine après build
add_custom_command(TARGET radar_scope POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:radar_scope>
        ${CMAKE_SOURCE_DIR}/radar_scope.exe
    COMMENT "Copie de l'exécutable dans pc_scope/"
)
