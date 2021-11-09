#include "boids.hpp"

#include "../../../vcl/math/transformation/transformation.hpp"
#include "../../../vcl/wrapper/perlin/perlin.hpp"

#ifdef SCENE_BOIDS

using namespace vcl;

static void set_gui(timer_basic &timer, float &angle);

float computeAngle(const vcl::vec3 &a, const vcl::vec3 &b)
{
    return acosf(dot(a, b) / (norm(a) + norm(b))) * 180 / M_PI;
}

void scene_model::setup_data(std::map<std::string, GLuint> &shaders,
                             scene_structure &, gui_structure &gui)
{
    std::vector<vec3> borders_segments = {
        { -1, -1, -1 }, { 1, -1, -1 }, { 1, -1, -1 }, { 1, 1, -1 },
        { 1, 1, -1 },   { -1, 1, -1 }, { -1, 1, -1 }, { -1, -1, -1 },
        { -1, -1, 1 },  { 1, -1, 1 },  { 1, -1, 1 },  { 1, 1, 1 },
        { 1, 1, 1 },    { -1, 1, 1 },  { -1, 1, 1 },  { -1, -1, 1 },
        { -1, -1, -1 }, { -1, -1, 1 }, { 1, -1, -1 }, { 1, -1, 1 },
        { 1, 1, -1 },   { 1, 1, 1 },   { -1, 1, -1 }, { -1, 1, 1 }
    };
    borders = borders_segments;
    borders.uniform.color = { 0, 0, 0 };

    gui.show_frame_camera = false;

    mates.reserve(n_mates);
    random_real_generator init_gen{ -1, 1 };
    for (int i = 0; i < n_mates; i++)
    {
        vcl::vec3 dir{ init_gen(), init_gen(), init_gen() };
        dir = vcl::normalize(dir);
        mates.push_back(std::make_shared<mate>(
            mate{ { init_gen(), init_gen(), init_gen() }, dir, 0.25, 0.5 }));
    }

    mate_mesh =
        mesh_drawable(mesh_primitive_cone(0.02, { 0, 0, 0 }, { 0.06, 0, 0 }));
    mate_mesh.shader = shaders["mesh"];
}

void scene_model::update_flock()
{
    float dt = 0.02f * timer.scale;
    for (auto &mate : mates)
    {
        mate->drawn = false;
        vec3 old_pos = mate->pos;
        vec3 random_dir_variation{ var_gen(), var_gen(), var_gen() };
        mate->findVisibleMates(mates, mate_view_angle);
        vec3 avoidance = mate->avoid();
        auto avoidance_norm = norm(avoidance);
        if (avoidance_norm != 0)
            avoidance /= avoidance_norm;

        vcl::vec3 new_dir =
            vcl::normalize(mate->dir + random_dir_variation + avoidance);
        vcl::vec3 new_pos = old_pos + dt * (mate->speed * new_dir);
        mate->dir = new_dir;
        mate->pos = new_pos;
    }
}

void scene_model::frame_draw(std::map<std::string, GLuint> &shaders,
                             scene_structure &scene, gui_structure &)
{
    timer.update();
    set_gui(timer, mate_view_angle);

    update_flock();
    mates[0]->draw_mate(mate_mesh, scene.camera, { 0.2, 0.3, 1 });
    for (const auto &mate : mates[0]->visibleMates)
        mate->draw_mate(mate_mesh, scene.camera, { 1., 0.2, 0.3 });
    for (const auto &mate : mates)
        mate->draw_mate(mate_mesh, scene.camera, { 0.2, 0.8, 1 });
    draw(borders, scene.camera, shaders["curve"]);
}

static void set_gui(timer_basic &timer, float &angle)
{
    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale,
                        &scale_min, &scale_max, "%.2f s");

    float angle_min = 60;
    float angle_max = 360;
    ImGui::SliderScalar("Mate FOV", ImGuiDataType_Float, &angle, &angle_min,
                        &angle_max, "%.f");

    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();
}

#endif