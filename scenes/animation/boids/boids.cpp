#include "boids.hpp"

#include "../../../vcl/math/transformation/transformation.hpp"
#include "../../../vcl/wrapper/perlin/perlin.hpp"

#ifdef SCENE_BOIDS

#    define CUBE_HALF 1

using namespace vcl;

static void set_gui(timer_basic &timer, float &fov_radius, float &angle,
                    float &avoidance_coeff, float &avoidance_radius_ratio,
                    float &alignment_coeff, float &alignment_radius_ratio,
                    float &cohesion_coeff);

float computeAngle(const vcl::vec3 &a, const vcl::vec3 &b)
{
    return acosf(dot(a, b) / (norm(a) + norm(b))) * 180 / M_PI;
}

void scene_model::setup_data(std::map<std::string, GLuint> &shaders,
                             scene_structure &, gui_structure &gui)
{
    cube_faces.resize(6);
    cube_faces[0].n = { 0, 1, 0 };
    cube_faces[0].a = { 0, -CUBE_HALF, 0 };
    cube_faces[1].n = { 0, -1, 0 };
    cube_faces[1].a = { 0, CUBE_HALF, 0 };
    cube_faces[2].n = { 1, 0, 0 };
    cube_faces[2].a = { -CUBE_HALF, 0, 0 };
    cube_faces[3].n = { -1, 0, 0 };
    cube_faces[3].a = { CUBE_HALF, 0, 0 };
    cube_faces[4].n = { 0, 0, 1 };
    cube_faces[4].a = { 0, 0, -CUBE_HALF };
    cube_faces[5].n = { 0, 0, -1 };
    cube_faces[5].a = { 0, 0, CUBE_HALF };
    std::vector<vec3> borders_segments = {
        { -CUBE_HALF, -CUBE_HALF, -CUBE_HALF },
        { CUBE_HALF, -CUBE_HALF, -CUBE_HALF },
        { CUBE_HALF, -CUBE_HALF, -CUBE_HALF },
        { CUBE_HALF, CUBE_HALF, -CUBE_HALF },
        { CUBE_HALF, CUBE_HALF, -CUBE_HALF },
        { -CUBE_HALF, CUBE_HALF, -CUBE_HALF },
        { -CUBE_HALF, CUBE_HALF, -CUBE_HALF },
        { -CUBE_HALF, -CUBE_HALF, -CUBE_HALF },
        { -CUBE_HALF, -CUBE_HALF, CUBE_HALF },
        { CUBE_HALF, -CUBE_HALF, CUBE_HALF },
        { CUBE_HALF, -CUBE_HALF, CUBE_HALF },
        { CUBE_HALF, CUBE_HALF, CUBE_HALF },
        { CUBE_HALF, CUBE_HALF, CUBE_HALF },
        { -CUBE_HALF, CUBE_HALF, CUBE_HALF },
        { -CUBE_HALF, CUBE_HALF, CUBE_HALF },
        { -CUBE_HALF, -CUBE_HALF, CUBE_HALF },
        { -CUBE_HALF, -CUBE_HALF, -CUBE_HALF },
        { -CUBE_HALF, -CUBE_HALF, CUBE_HALF },
        { CUBE_HALF, -CUBE_HALF, -CUBE_HALF },
        { CUBE_HALF, -CUBE_HALF, CUBE_HALF },
        { CUBE_HALF, CUBE_HALF, -CUBE_HALF },
        { CUBE_HALF, CUBE_HALF, CUBE_HALF },
        { -CUBE_HALF, CUBE_HALF, -CUBE_HALF },
        { -CUBE_HALF, CUBE_HALF, CUBE_HALF }
    };
    borders = segments_gpu(borders_segments);
    borders.uniform.color = { 0, 0, 0 };
    borders.shader = shaders["curve"];

    gui.show_frame_camera = false;

    cur_mates = std::make_shared<flock>();
    next_mates = std::make_shared<flock>();
    cur_mates->reserve(n_mates);
    next_mates->reserve(n_mates);
    random_real_generator init_gen{ -CUBE_HALF + 0.2, CUBE_HALF - 0.2 };
    for (int i = 0; i < n_mates; i++)
    {
        vcl::vec3 pos{ init_gen(), init_gen(), init_gen() };
        vcl::vec3 dir{ init_gen(), init_gen(), init_gen() };
        dir = vcl::normalize(dir);
        cur_mates->push_back(
            std::make_shared<mate>(mate{ pos, dir, 0.5, fov_radius }));
        next_mates->push_back(
            std::make_shared<mate>(mate{ pos, dir, 0.5, fov_radius }));
    }

    mate_mesh =
        mesh_drawable(mesh_primitive_cone(0.02, { 0, 0, 0 }, { 0.06, 0, 0 }));
    mate_mesh.shader = shaders["mesh"];
}

void scene_model::update_flock()
{
    float dt = 0.02f * timer.scale;
    auto &cur = *cur_mates;
    auto &next = *next_mates;

    auto thread_pool = std::vector<std::thread>();
    thread_pool.reserve(nb_threads);
    auto lambda = [&](size_t i) {
        random_real_generator var_gen{ -RANDOM_VARIATION, RANDOM_VARIATION };
        for (size_t j = i; j < cur.size(); j += nb_threads)
        {
            const auto &mate = cur[j];
            auto &next_mate = next[j];

            vec3 random_dir_variation{ var_gen(), var_gen(), var_gen() };
            mate->findVisibleMates(cur, mate_view_angle);
            vec3 mate_avoidance_dir = mate->avoid_mates(avoidance_radius_ratio);
            vec3 wall_avoidance_dir =
                mate->avoid_walls(avoidance_radius_ratio, cube_faces);
            vec3 alignment_dir = mate->alignment(alignment_radius_ratio);
            vec3 cohesion_dir = mate->cohesion();

            vec3 new_dir = normalize(
                mate->dir + random_dir_variation + 0.075f * wall_avoidance_dir
                + avoidance_coeff * 0.05f * mate_avoidance_dir
                + alignment_coeff * 0.05f * alignment_dir
                + cohesion_coeff * 0.05f * cohesion_dir);
            vec3 new_pos = mate->pos + dt * (mate->speed * new_dir);

            next_mate->drawn = false;
            next_mate->dir = new_dir;
            next_mate->pos = new_pos;
        }
    };

    for (size_t i = 0; i < nb_threads; ++i)
        thread_pool.emplace_back(lambda, i);
    for (auto &t : thread_pool)
        t.join();

    cur_mates.swap(next_mates);
}

void scene_model::frame_draw(std::map<std::string, GLuint> &shaders,
                             scene_structure &scene, gui_structure &)
{
    timer.update();
    set_gui(timer, fov_radius, mate_view_angle, avoidance_coeff,
            avoidance_radius_ratio, alignment_coeff, alignment_radius_ratio,
            cohesion_coeff);

    update_flock();
    const auto &cur = *cur_mates;

    /*
    auto &subject_0 = *cur[0];
    subject_0.draw_mate(mate_mesh, scene.camera, { 0.2, 0.3, 1 });
    for (const auto &mate : subject_0.visibleMates)
    {
        auto distToMate = norm(mate->pos - subject_0.pos);
        if (distToMate < subject_0.fov_radius * avoidance_radius_ratio)
            mate->draw_mate(mate_mesh, scene.camera, { 1., 0.2, 0.3 });
        else if (distToMate < subject_0.fov_radius)
            mate->draw_mate(mate_mesh, scene.camera, { 0.2, 1., 0.3 });
    }
    */

    for (const auto &mate : cur)
        mate->draw_mate(mate_mesh, scene.camera, { 0.2, 0.8, 1 });

    draw(borders, scene.camera, shaders["curve"]);
}

static void set_gui(timer_basic &timer, float &fov_radius, float &angle,
                    float &avoidance_coeff, float &avoidance_radius_ratio,
                    float &alignment_coeff, float &alignment_radius_ratio,
                    float &cohesion_coeff)
{
    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale,
                        &scale_min, &scale_max, "%.2f s");

    float fov_radius_min = 0.1f;
    float fov_radius_max = 2.f;
    ImGui::SliderScalar("Mate FOV Radius", ImGuiDataType_Float, &fov_radius,
                        &fov_radius_min, &fov_radius_max, "%.2f");

    float angle_min = 60;
    float angle_max = 360;
    ImGui::SliderScalar("Mate FOV Angle", ImGuiDataType_Float, &angle,
                        &angle_min, &angle_max, "%.f");

    float avoidance_min = 0.f;
    float avoidance_max = 1.f;
    ImGui::SliderScalar("Avoidance Strength", ImGuiDataType_Float,
                        &avoidance_coeff, &avoidance_min, &avoidance_max,
                        "%.2f");

    float avoidance_radius_min = 0.f;
    float avoidance_radius_max = 1.f;
    ImGui::SliderScalar("Avoidance Radius Ratio", ImGuiDataType_Float,
                        &avoidance_radius_ratio, &avoidance_radius_min,
                        &avoidance_radius_max, "%.2f");

    float alignment_min = 0.f;
    float alignment_max = 1.f;
    ImGui::SliderScalar("Alignment Strength", ImGuiDataType_Float,
                        &alignment_coeff, &alignment_min, &alignment_max,
                        "%.2f");

    float alignment_radius_min = 0.f;
    float alignment_radius_max = 1.f;
    ImGui::SliderScalar("Alignment Radius Ratio", ImGuiDataType_Float,
                        &alignment_radius_ratio, &alignment_radius_min,
                        &alignment_radius_max, "%.2f");

    float cohesion_min = 0.f;
    float cohesion_max = 1.f;
    ImGui::SliderScalar("Cohesion Strength", ImGuiDataType_Float,
                        &cohesion_coeff, &cohesion_min, &cohesion_max, "%.2f");

    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();
}

#endif