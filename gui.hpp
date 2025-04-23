#ifndef GUI_HPP
#define GUI_HPP

// --- ADDED SDL Includes ---
#include <SDL.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif
// --- End Added SDL Includes ---

// Forward declarations to minimize header dependencies
namespace DJI {
    namespace OSDK {
        class Vehicle; // From dji_vehicle.hpp
        // class LinuxSetup; // <-- REMOVED Forward declaration
    }
}

/**
 * @brief Initializes SDL, GLEW, and ImGui. Creates the main window and OpenGL context.
 *
 * @param window Pointer to an SDL_Window* which will be created.
 * @param gl_context Pointer to an SDL_GLContext* which will be created.
 * @return true on successful initialization, false otherwise.
 */
bool initializeGui(SDL_Window*& window, SDL_GLContext& gl_context);

/**
 * @brief Runs the main GUI loop. Handles events, renders ImGui windows, and manages GUI state.
 *        This function will block until the GUI is closed.
 *
 * @param window The main SDL window.
 * @param gl_context The OpenGL context.
 * @param vehicle Pointer to the initialized DJI Vehicle object (can be nullptr).
 * @param enableFlightControl Flag indicating if flight control is available.
 */
void runGui(SDL_Window* window, SDL_GLContext gl_context, DJI::OSDK::Vehicle* vehicle, bool enableFlightControl);

/**
 * @brief Cleans up and shuts down ImGui, SDL, and GLEW resources.
 *
 * @param window The main SDL window.
 * @param gl_context The OpenGL context.
 */
void cleanupGui(SDL_Window* window, SDL_GLContext gl_context);

/**
 * @brief Checks if the processing thread is running and signals it to stop if it is.
 *        Joins the thread and resets relevant global states (flags, data vectors, status).
 */
void stopProcessingThreadIfNeeded();


#endif // GUI_HPP