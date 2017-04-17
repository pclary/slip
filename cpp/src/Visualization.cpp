#include "Visualization.hpp"


size_t Visualization::instances = 0;
const mjModel* Visualization::m = nullptr;


Visualization::Visualization(const mjModel* m)
{
    // Initialize GLFW if this is the first instance
    if (!instances) {
        if (!glfwInit())
            mju_error("Could not initialize GLFW");
    }

    // Save model pointer
    this->m = m;

    // Create window
    window = glfwCreateWindow(1200, 900, "Cassie", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Set up mujoco visualization objects
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjv_makeScene(&scn, 1000);
    mjr_makeContext(m, &con, mjFONTSCALE_100);

    // Count instances for GLFW initialization/termination
    ++instances;
}



Visualization::~Visualization()
{
    // Count instances for GLFW initialization/termination
    --instances;

    // Free mujoco objects
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // Terminate GLFW if this is the last instance
    if (!instances)
        glfwTerminate();
}


bool Visualization::update(mjData* d)
{
    // Return early if window is closed
    if (!window)
        return false;

    // Handle closing window
    if (glfwWindowShouldClose(window)) {
        glfwDestroyWindow(window);
        window = nullptr;
        return false;
    }

    // Set up for rendering
    glfwMakeContextCurrent(window);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // Render scene
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // Show updated scene
    glfwSwapBuffers(window);
    glfwPollEvents();

    return true;
}
