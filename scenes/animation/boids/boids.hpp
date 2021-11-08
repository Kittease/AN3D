#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_BOIDS

struct mate {
    vcl::vec3 pos;
    vcl::vec3 speed;
};

struct scene_model : scene_base {
    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    int n_mates = 100;
    std::vector<mate> flock;
    vcl::mesh_drawable mate_mesh;

    vcl::segments_drawable borders;

    vcl::timer_event timer;
};

#endif