#pragma once

#include <array>
#include <vector>


struct terrain_t {};

typedef std::array<std::vector<double>, 2> ground_data_t;


terrain_t get_local_terrain(const ground_data_t& gd, double x);
