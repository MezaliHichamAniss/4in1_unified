cmake_minimum_required(VERSION 3.20)
project(4in1_unified VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Output directories ────────────────────────────────────────────────────────
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ── Options ───────────────────────────────────────────────────────────────────
option(USE_WINRT_BLE "Enable WinRT BLE layer (Windows only)" OFF)

# ── Find OpenCV (user installs separately) ────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc video videoio)

# ── Find OpenGL ───────────────────────────────────────────────────────────────
find_package(OpenGL REQUIRED)

# ── FetchContent dependencies ─────────────────────────────────────────────────
include(FetchContent)

# Eigen3
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
  GIT_SHALLOW    TRUE
)
set(EIGEN_BUILD_DOC       OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING         OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

# GLFW
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG        3.3.9
  GIT_SHALLOW    TRUE
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# ImGui
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.91.0
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# ImPlot
FetchContent_Declare(
  implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG        v0.17
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(implot)

# ── ImGui library target (no CMakeLists in imgui, compile manually) ───────────
add_library(imgui_lib STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw OpenGL::GL)

# ── ImPlot library target ─────────────────────────────────────────────────────
add_library(implot_lib STATIC
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(implot_lib PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot_lib PUBLIC imgui_lib)

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/cdp_state.cpp
  src/imm_tracker.cpp
  src/controllers.cpp
  src/vision.cpp
  src/ble_layer.cpp
  src/logger.cpp
  src/gui.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${OpenCV_LIBS}
  Eigen3::Eigen
  imgui_lib
  implot_lib
  OpenGL::GL
)

# ── Windows-specific: WinRT BLE ───────────────────────────────────────────────
if(WIN32 AND USE_WINRT_BLE)
  target_compile_definitions(${PROJECT_NAME} PRIVATE _WIN32_WINNT=0x0A00)
  target_link_libraries(${PROJECT_NAME} PRIVATE windowsapp)
endif()

# ── Platform threads ──────────────────────────────────────────────────────────
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

# ── Compiler warnings ─────────────────────────────────────────────────────────
if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W3)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
