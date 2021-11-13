#pragma once

#include <memory>
#include <thread>

#include "main/scene_base/base.hpp"

#ifdef SCENE_BOIDS
#    define RANDOM_VARIATION 0.025f

struct mate;
using flock = std::vector<std::shared_ptr<mate>>;

float computeAngle(const vcl::vec3 &a, const vcl::vec3 &b);

struct plane
{
    vcl::vec3 n;
    vcl::vec3 a;
};

struct update_vectors
{
    vcl::vec3 mate_avoid;
    vcl::vec3 alignment;
    vcl::vec3 cohesion;
    vcl::vec3 color;
};

struct mate
{
    vcl::vec3 pos;
    vcl::vec3 dir;
    float speed;

    float &fov_radius;
    bool drawn = false;
    flock visibleMates;

    vcl::vec3 base_color;
    vcl::vec3 color;
    float infectivity;

    mate(vcl::vec3 pos, vcl::vec3 dir, float speed, float &fov_radius,
         vcl::vec3 color, float infectivity)
        : pos{ pos }
        , dir{ dir }
        , speed{ speed }
        , fov_radius{ fov_radius }
        , base_color{ color }
        , color{ color }
        , infectivity{ infectivity }
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

    update_vectors compute_update(float avoid_ratio,
                                  float alignment_ratio) const
    {
        vcl::vec3 mate_avoid{ 0, 0, 0 };
        vcl::vec3 alignment{ 0, 0, 0 };
        vcl::vec3 center{ 0, 0, 0 };
        vcl::vec3 color{ 0, 0, 0 };
        float weights = 0;

        if (visibleMates.empty())
            return { mate_avoid, alignment, center, base_color };

        for (const auto &mate : visibleMates)
        {
            color += mate->color * mate->infectivity;
            weights += mate->infectivity;

            auto &mate_pos = mate->pos;
            center += mate_pos;

            auto curToMate = mate_pos - pos;
            auto strength = vcl::norm(curToMate);
            if (strength < fov_radius * alignment_ratio)
            {
                alignment += mate->dir;
                if (strength < fov_radius * avoid_ratio)
                    mate_avoid -= curToMate / (strength * strength);
            }
        }

        // Colouring
        color =
            (this->color * infectivity) + (color / weights) * (1 - infectivity);

        // Cohesion
        center /= visibleMates.size();
        auto center_dir = center - pos;
        auto center_dir_norm = norm(center_dir);
        if (center_dir_norm != 0)
            center_dir /= center_dir_norm;

        // Separation
        auto mate_avoid_norm = norm(mate_avoid);
        if (mate_avoid_norm != 0)
            mate_avoid /= mate_avoid_norm;

        // Alignment
        auto alignment_norm = norm(alignment);
        if (alignment_norm != 0)
            alignment /= alignment_norm;

        return { mate_avoid, alignment, center_dir, color };
    }

    vcl::vec3 avoid_walls(float avoid_ratio,
                          const vcl::buffer<plane> &faces) const
    {
        vcl::vec3 f{ 0, 0, 0 };
        for (const auto &face : faces)
        {
            float collision = dot(pos - face.a, face.n);
            if (collision <= fov_radius * avoid_ratio)
                f += face.n / (collision * collision);
        }

        auto f_norm = norm(f);
        return f_norm == 0 ? f : f / f_norm;
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

    const int n_mates = 500;
    const unsigned short nb_threads = 6;
    float mate_view_angle = 180;
    float fov_radius = 0.35f;
    float avoidance_radius_ratio = 0.25f;
    float avoidance_coeff = 0.5f;
    float alignment_radius_ratio = 0.5f;
    float alignment_coeff = 0.5f;
    float cohesion_coeff = 0.5f;
    bool debug_mode = false;

    std::shared_ptr<flock> cur_mates;
    std::shared_ptr<flock> next_mates;
    vcl::mesh_drawable mate_mesh;

    vcl::buffer<plane> cube_faces;
    vcl::segments_drawable borders;

    vcl::timer_event timer;
};

#endif