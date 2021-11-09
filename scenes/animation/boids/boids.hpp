#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_BOIDS
#    define RANDOM_VARIATION 0.5f

struct mate
{
    vcl::vec3 pos;
    vcl::vec3 dir;
    float speed;
};

class random_real_generator
{
public:
    random_real_generator(float a, float b)
    {
        gen = std::mt19937(rd());
        dis = std::uniform_real_distribution<float>(a, b);
    }

    float operator()()
    {
        return dis(gen);
    }

protected:
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;
};

struct scene_model : scene_base
{
    void setup_data(std::map<std::string, GLuint> &shaders,
                    scene_structure &scene, gui_structure &gui);
    void frame_draw(std::map<std::string, GLuint> &shaders,
                    scene_structure &scene, gui_structure &gui);

    int n_mates = 100;
    std::vector<mate> flock;
    vcl::mesh_drawable mate_mesh;

    vcl::segments_drawable borders;

    vcl::timer_event timer;
    random_real_generator var_gen{ -RANDOM_VARIATION, RANDOM_VARIATION };
};

#endif