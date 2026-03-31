cmake_minimum_required(VERSION 3.20)
project(radar_analyzer LANGUAGES CXX)

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

# ── Dear ImGui v1.91.8 ──
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# Bibliothèque ImGui statique
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
add_executable(radar_analyzer radar_analyzer.cpp)

target_link_libraries(radar_analyzer PRIVATE
    imgui_lib
    glfw
    opengl32      # Win32 OpenGL
    ws2_32        # Winsock2 (UDP)
    gdi32         # MinGW
    imm32
    dwmapi
)

target_compile_definitions(radar_analyzer PRIVATE
    _CRT_SECURE_NO_WARNINGS
    _USE_MATH_DEFINES
    _WINSOCK_DEPRECATED_NO_WARNINGS
)

# Optimisations
if(MSVC)
    target_compile_options(radar_analyzer PRIVATE /O2 /W3)
else()
    target_compile_options(radar_analyzer PRIVATE -O2 -Wall)
endif()

# Copier l'exécutable dans le dossier racine après build
add_custom_command(TARGET radar_analyzer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:radar_analyzer>
        ${CMAKE_SOURCE_DIR}/radar_analyzer.exe
    COMMENT "Copie de l'exécutable dans pc_analyzer/"
)

# Copier les DLL runtime MinGW/MSYS2 nécessaires à côté de l'exécutable.
# Cela permet un lancement par double-clic sans dépendre du PATH système.
if (MINGW)
    set(MSYS2_UCRT_BIN "C:/msys64/ucrt64/bin")
    add_custom_command(TARGET radar_analyzer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MSYS2_UCRT_BIN}/libstdc++-6.dll"
            "${MSYS2_UCRT_BIN}/libgcc_s_seh-1.dll"
            "${MSYS2_UCRT_BIN}/libwinpthread-1.dll"
            "${MSYS2_UCRT_BIN}/libiconv-2.dll"
            "${MSYS2_UCRT_BIN}/libintl-8.dll"
            "$<TARGET_FILE_DIR:radar_analyzer>"
        COMMENT "Copie des DLL runtime MinGW/MSYS2"
    )

    add_custom_command(TARGET radar_analyzer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MSYS2_UCRT_BIN}/libstdc++-6.dll"
            "${MSYS2_UCRT_BIN}/libgcc_s_seh-1.dll"
            "${MSYS2_UCRT_BIN}/libwinpthread-1.dll"
            "${MSYS2_UCRT_BIN}/libiconv-2.dll"
            "${MSYS2_UCRT_BIN}/libintl-8.dll"
            "${CMAKE_SOURCE_DIR}"
        COMMENT "Copie des DLL runtime vers pc_analyzer/"
    )
endif()
