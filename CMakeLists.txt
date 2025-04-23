cmake_minimum_required(VERSION 3.10) # Increased minimum version for better find_package support
project(djiosdk-flightcontrol-sample)

set(CMAKE_CXX_STANDARD 11) # Explicitly set C++ standard
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g -O0") # Added -pthread for std::thread, -g for debug, -O0 for no optimization

# --- OSDK Includes ---
# Assuming this path is correct relative to your build directory or source directory context
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../core/inc)

# --- Vendor Includes ---
# Add vendor directory for ImGui includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/backends)
# Add include directory for nlohmann/json (header-only)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/vendor/nlohmann) # Adjust if json.hpp is elsewhere


# --- Find Dependencies ---

# Find pybind11 for Python integration (Keep as requested, though usage seems indirect)
# Consider using find_package(PythonLibs REQUIRED) as well if you embed Python
# list(APPEND CMAKE_PREFIX_PATH "/home/ricky/.local/lib/python3.11/site-packages/pybind11/share/cmake/pybind11") # User-specific path, might need adjustment
# find_package(pybind11 REQUIRED)

# Find Boost (for ASIO used in python_bridge.cpp)
find_package(Boost 1.71 REQUIRED COMPONENTS system) # Require Boost system library, adjust version if needed
include_directories(${Boost_INCLUDE_DIRS})

# Find SDL2
find_package(SDL2 REQUIRED)
# Include SDL2 directories AFTER finding the package
include_directories(${SDL2_INCLUDE_DIRS})

# Find OpenGL (Ensures a basic GL implementation is available)
find_package(OpenGL REQUIRED)
# OpenGL include directories are usually handled by the system or find_package

# Find GLEW (OpenGL Loader)
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS}) # Add GLEW include path

# Find Threads (already using -pthread, but good practice)
find_package(Threads REQUIRED)

# --- Gather Source Files ---

# OSDK and Common Source Files (Keep as provided)
FILE(GLOB OSDK_COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../core/src/flight_sample.cpp # Check if this is needed
    ${CMAKE_CURRENT_SOURCE_DIR}/../hal/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../osal/*.c
    # ${LINKER_HEADER_SRC} # Commented out if not defined/used
)

if (OSDK_HOTPLUG)
    FILE(GLOB OSDK_HOTPLUG_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../hal/hotplug/*.c)
endif ()

# Main application source file
set(MAIN_SOURCE main.cpp)

# New Project Source Files (Refactored Code)
set(PROJECT_SOURCES
    config.cpp
    dji_handler.cpp
    globals.cpp
    gui.cpp
    logging.cpp
    python_bridge.cpp
    radar_processor.cpp
    utils.cpp
)

# ImGui Source Files (Keep as provided)
set(IMGUI_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/imgui.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/imgui_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/imgui_tables.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/imgui_widgets.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_sdl2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_opengl3.cpp
)


# Combine all source files
set(ALL_SOURCE_FILES
    ${MAIN_SOURCE}
    ${PROJECT_SOURCES}          # Add the new project sources
    ${OSDK_COMMON_SOURCES}
    ${OSDK_HOTPLUG_SOURCES}     # This will be empty if OSDK_HOTPLUG is false
    ${IMGUI_SOURCES}
)

# --- Add Executable Target ---
add_executable(${PROJECT_NAME} ${ALL_SOURCE_FILES})

# --- Link Libraries ---
target_link_libraries(${PROJECT_NAME} PRIVATE
    # pybind11::embed       # Keep commented unless direct embedding is used
    ${SDL2_LIBRARIES}     # Link against SDL2
    OpenGL::GL            # Link against system OpenGL (needed by SDL/ImGui)
    GLEW::glew            # Link against GLEW (or GLEW::glew_s if static)
    Threads::Threads      # Link against pthreads
    Boost::system         # Link against Boost system library (for ASIO)
)

# --- Optional: Define preprocessor flags if needed by ImGui backends ---
# Define which GL loader ImGui should expect (important!)
target_compile_definitions(${PROJECT_NAME} PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLEW)
# If linking GLEW statically, you might also need:
# target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)


# --- Output ---
message(STATUS "OSDK Common Sources: ${OSDK_COMMON_SOURCES}")
message(STATUS "OSDK Hotplug Sources: ${OSDK_HOTPLUG_SOURCES}")
message(STATUS "ImGui Sources: ${IMGUI_SOURCES}")
message(STATUS "Project Sources: ${PROJECT_SOURCES}") # Added message for new sources
message(STATUS "Main Source: ${MAIN_SOURCE}")
message(STATUS "SDL2 Include Dirs: ${SDL2_INCLUDE_DIRS}")
message(STATUS "SDL2 Libraries: ${SDL2_LIBRARIES}")
message(STATUS "OpenGL Libraries: ${OpenGL_LIBRARIES}")
message(STATUS "GLEW Include Dirs: ${GLEW_INCLUDE_DIRS}")
message(STATUS "GLEW Libraries: ${GLEW_LIBRARIES}") # GLEW::glew target is preferred
message(STATUS "Boost Include Dirs: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost Libraries: ${Boost_LIBRARIES}")