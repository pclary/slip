#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <cstddef>
#include "mujoco.h"
#include "glfw3.h"


class Visualization {
public:
    Visualization(const mjModel* m);
    Visualization(const Visualization&) = delete;
    Visualization& operator=(const Visualization&) = delete;
    Visualization(Visualization&&) = default;
    Visualization& operator=(Visualization&&) = default;
    ~Visualization();

    bool update(mjData* d);

private:
    static size_t instances;
    static const mjModel* m;

    GLFWwindow* window;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

};


#endif // VISUALIZATION_HPP
