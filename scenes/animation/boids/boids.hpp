#pragma once

#include <memory>

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

struct mate
{
    vcl::vec3 pos;
    vcl::vec3 dir;
    float speed;

    float &fov_radius;
    bool drawn = false;
    flock visibleMates;

    mate(vcl::vec3 pos, vcl::vec3 dir, float speed, float &fov_radius)
        : pos{ pos }
        , dir{ dir }
        , speed{ speed }
        , fov_radius{ fov_radius }
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

    vcl::vec3 avoid_mates(float avoid_ratio)
    {
        vcl::vec3 f{ 0, 0, 0 };
        for (const auto &mate : visibleMates)
        {
            auto curToMate = mate->pos - pos;
            auto strength = vcl::norm(curToMate);
            if (strength <= fov_radius * avoid_ratio)
            {
                // f -= curToMate;
                // vcl::vec3 dn = curToMate / strength * dot(dir, curToMate);
                // vcl::vec3 dt = dir - dn;
                // f += 0.7 * dt - 0.3 * dn;
                f -= curToMate / (strength * strength);
            }
        }
        auto f_norm = norm(f);
        return f_norm == 0 ? f : f / f_norm;
    }

    vcl::vec3 avoid_walls(float avoid_ratio, const vcl::buffer<plane> &faces)
    {
        vcl::vec3 f{ 0, 0, 0 };
        for (const auto &face : faces)
        {
            float collision = dot(pos - face.a, face.n);
            if (collision <= fov_radius * avoid_ratio)
            {
                // float d = avoid_radius - collision;
                // pos += d * face.n;
                // vcl::vec3 dn = face.n / collision * dot(dir, face.n);
                // vcl::vec3 dt = dir - dn;
                // f += 0.7 * dt - 0.3 * dn;
                f += face.n / (collision * collision);
            }
        }

        auto f_norm = norm(f);
        return f_norm == 0 ? f : f / f_norm;
    }

    vcl::vec3 cohesion()
    {
        vcl::vec3 center{ 0, 0, 0 };
        for (const auto &mate : visibleMates)
            center += mate->pos;
        if (!visibleMates.empty())
            center /= visibleMates.size();

        auto center_dir = center - pos;
        auto center_dir_norm = norm(center_dir);
        return center_dir_norm == 0 ? center_dir : center_dir / center_dir_norm;
    }

    vcl::vec3 alignment(float alignment_ratio)
    {
        vcl::vec3 mate_dir{ 0, 0, 0 };
        for (const auto &mate : visibleMates)
            if (norm(mate->pos - pos) < fov_radius * alignment_ratio)
                mate_dir += mate->dir;

        auto mate_dir_norm = norm(mate_dir);
        return mate_dir_norm == 0 ? mate_dir : mate_dir / mate_dir_norm;
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

    int n_mates = 500;
    float mate_view_angle = 180;
    float fov_radius = 0.5f;
    float avoidance_radius_ratio = 0.25f;
    float avoidance_coeff = 0.5f;
    float alignment_radius_ratio = 0.5f;
    float alignment_coeff = 0.5f;
    float cohesion_coeff = 0.5f;

    std::shared_ptr<flock> cur_mates;
    std::shared_ptr<flock> next_mates;
    vcl::mesh_drawable mate_mesh;

    vcl::buffer<plane> cube_faces;
    vcl::segments_drawable borders;

    vcl::timer_event timer;
    random_real_generator var_gen{ -RANDOM_VARIATION, RANDOM_VARIATION };
};

#endif