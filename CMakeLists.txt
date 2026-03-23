cmake_minimum_required(VERSION 3.20)
project(4in1_unified LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── OpenCV ──────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED HINTS "C:/opencv/build/x64/vc16/lib")

# ── Eigen (header-only, fetched automatically if not found) ─────────────────
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
        GIT_SHALLOW    TRUE
    )
    set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_DOC     OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(eigen)
    set(EIGEN3_INCLUDE_DIR ${eigen_SOURCE_DIR})
else()
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

# ── nlohmann/json (header-only, fetched automatically if not found) ──────────
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.3
        GIT_SHALLOW    TRUE
    )
    set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(json)
endif()

# ── Sources ─────────────────────────────────────────────────────────────────
set(SOURCES
    src/main.cpp
    src/controllers.cpp
    src/vision.cpp
    src/ble.cpp
    src/logging.cpp
    src/gui.cpp
)

add_executable(4in1_unified ${SOURCES})

target_include_directories(4in1_unified PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(4in1_unified PRIVATE
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

# ── Windows-specific: WinRT / BLE ───────────────────────────────────────────
if(WIN32)
    target_compile_options(4in1_unified PRIVATE /await /EHsc /W3)
    target_link_libraries(4in1_unified PRIVATE
        WindowsApp.lib
        runtimeobject.lib
    )
    # WinRT C++/WinRT projection headers come with the Windows SDK 10.0.17763+
    target_compile_definitions(4in1_unified PRIVATE
        WINRT_LEAN_AND_MEAN
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        _WIN32_WINNT=0x0A00  # Windows 10
    )
endif()

# ── Release optimisations ───────────────────────────────────────────────────
if(MSVC)
    target_compile_options(4in1_unified PRIVATE $<$<CONFIG:Release>:/O2 /GL>)
    target_link_options(4in1_unified PRIVATE $<$<CONFIG:Release>:/LTCG>)
endif()
