#pragma once

#include <memory>

#include "main/scene_base/base.hpp"

#ifdef SCENE_BOIDS
#    define RANDOM_VARIATION 0.3f

struct mate;
using flock = std::vector<std::shared_ptr<mate>>;

float computeAngle(const vcl::vec3 &a, const vcl::vec3 &b);

struct mate
{
    vcl::vec3 pos;
    vcl::vec3 dir;
    float speed;

    float fov_radius;
    float avoid_radius;
    bool drawn = false;
    flock visibleMates;

    mate(vcl::vec3 pos, vcl::vec3 dir, float speed, float fov_radius)
        : pos{ pos }
        , dir{ dir }
        , speed{ speed }
        , fov_radius{ fov_radius }
        , avoid_radius{ fov_radius / 3 }
    {}

    void draw_mate(vcl::mesh_drawable &mate_mesh, vcl::camera_scene &camera,
                   vcl::vec3 color)
    {
        if (!drawn)
        {
            mate_mesh.uniform.transform.translation = pos;
            mate_mesh.uniform.transform.rotation =
                rotation_between_vector_mat3({ 1, 0, 0 }, dir);

            mate_mesh.uniform.color = color;
            draw(mate_mesh, camera);
            drawn = true;
        }
    }

    void findVisibleMates(const flock &mates, float angle)
    {
        visibleMates.clear();
        for (const auto &mate : mates)
        {
            if (mate.get() == this)
                continue;
            auto curToMate = mate->pos - pos;
            if (vcl::norm(curToMate) < fov_radius
                && computeAngle(dir, curToMate) < angle / 2)
                visibleMates.emplace_back(mate);
        }
    }

    vcl::vec3 avoid()
    {
        vcl::vec3 f{ 0, 0, 0 };
        for (const auto &mate : visibleMates)
        {
            auto curToMate = mate->pos - pos;
            auto strength = vcl::norm(curToMate);
            if (strength < avoid_radius)
                f -= curToMate;
        }

        return f;
    }
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
    void update_flock();

    int n_mates = 300;
    float mate_view_angle = 180;
    flock mates;
    vcl::mesh_drawable mate_mesh;

    vcl::segments_drawable borders;

    vcl::timer_event timer;
    random_real_generator var_gen{ -RANDOM_VARIATION, RANDOM_VARIATION };
};

#endif