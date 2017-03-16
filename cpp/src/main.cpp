#include <iostream>
#include <cstddef>
#include <unistd.h>
#include "Planner.hpp"
#include "Simulation.hpp"
#include "Controller.hpp"
#include "mujoco.h"
#include "glfw3.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


int main(void)
{
    // activate software
    mj_activate("mjkey.txt");

    // Load model
    char error[1000] = "Could not load binary model";
    m = mj_loadXML("planar.xml", 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    std::cout << m->nu << std::endl;

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    // mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = 0;
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjv_makeScene(&scn, 1000);                   // space for 1000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_100);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);

    controller_state_t cstate;
    controller_params_t cparams;
    cparams.target_dx = 0.4;

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which
        //  it usually can, this loop will finish on time for the next
        //  frame to be rendered at 60 fps.  Otherwise add a cpu timer
        //  and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while(d->time - simstart < 1.0/60.0) {

            // Apply control inputs
            // d->ctrl[0] = u.right.angle_pos*0;
            // d->ctrl[1] = u.right.angle_vel*0;
            // d->ctrl[2] = u.right.length_pos*0;
            // d->ctrl[3] = u.right.length_vel*0;
            // d->ctrl[4] = u.left.angle_pos*0;
            // d->ctrl[5] = u.left.angle_vel*0;
            // d->ctrl[6] = u.left.length_pos*0;
            // d->ctrl[7] = u.left.length_vel*0;

            mj_step1(m, d);

            // Update controller
            auto X = get_robot_state(d);
            // printf("[%7.4g %7.4g %7.4g %7.4g %7.4g %7.4g %7.4g]\n", X.qpos[0], X.qpos[1], X.qpos[2], X.qpos[3], X.qpos[4], X.qpos[5], X.qpos[6]);
            auto u = step(X, cstate, cparams, m->opt.timestep);
            d->ctrl[0] =
                u.right.angle_kp * (u.right.angle_pos - X.qpos[RIGHT_THETA_EQ]) +
                u.right.angle_kv * (u.right.angle_vel - X.qvel[RIGHT_DTHETA_EQ]);
            d->ctrl[1] =
                u.right.length_kp * (u.right.length_pos - X.qpos[RIGHT_L_EQ]) +
                u.right.length_kv * (u.right.length_vel - X.qvel[RIGHT_DL_EQ]);
            d->ctrl[2] =
                u.left.angle_kp * (u.left.angle_pos - X.qpos[LEFT_THETA_EQ]) +
                u.left.angle_kv * (u.left.angle_vel - X.qvel[LEFT_DTHETA_EQ]);
            d->ctrl[3] =
                u.left.length_kp * (u.left.length_pos - X.qpos[LEFT_L_EQ]) +
                u.left.length_kv * (u.left.length_vel - X.qvel[LEFT_DL_EQ]);
            printf("t %9.4f\n", d->time);
            printf("X [%7.4g %7.4g %7.4g %7.4g]\n", X.qpos[RIGHT_THETA_EQ], X.qpos[RIGHT_L_EQ], X.qpos[LEFT_THETA_EQ], X.qpos[LEFT_L_EQ]);
            printf("dX [%7.4g %7.4g %7.4g]\n", X.qvel[BODY_DX], X.qvel[BODY_DY], X.qvel[BODY_DTHETA]);
            printf("r [%7.4g %7.4g %7.4g %7.4g]\n", u.right.angle_pos, u.right.length_pos, u.left.angle_pos, u.left.length_pos);
            printf("u [%7.4g %7.4g %7.4g %7.4g]\n", d->ctrl[0], d->ctrl[1], d->ctrl[2], d->ctrl[3]);

            // Update simulation
            mj_step2(m, d);
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // close GLFW, free visualization storage
    glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    return 1;
}
