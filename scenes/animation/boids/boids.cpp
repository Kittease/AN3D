#include "boids.hpp"

#include "../../../vcl/math/transformation/transformation.hpp"
#include "../../../vcl/wrapper/perlin/perlin.hpp"

#ifdef SCENE_BOIDS

using namespace vcl;

static void set_gui(timer_basic &timer);

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

    flock.resize(n_mates);
    random_real_generator init_gen{ -1, 1 };
    for (int i = 0; i < n_mates; i++)
    {
        vcl::vec3 dir{ init_gen(), init_gen(), init_gen() };
        dir = vcl::normalize(dir);
        flock[i] = mate{ { init_gen(), init_gen(), init_gen() }, dir, 0.25 };
    }

    mate_mesh =
        mesh_drawable(mesh_primitive_cone(0.02, { 0, 0, 0 }, { 0.06, 0, 0 }));
    mate_mesh.shader = shaders["mesh"];
}

void scene_model::frame_draw(std::map<std::string, GLuint> &shaders,
                             scene_structure &scene, gui_structure &)
{
    float dt = 0.02f * timer.scale;
    timer.update();
    set_gui(timer);

    draw(borders, scene.camera, shaders["curve"]);

    for (int i = 0; i < n_mates; i++)
    {
        vcl::vec3 dir = flock[i].dir;
        vcl::vec3 position = flock[i].pos;

        vcl::vec3 old_pos = flock[i].pos;
        vcl::vec3 random_dir_variation{ var_gen(), var_gen(), var_gen() };
        vcl::vec3 new_dir = vcl::normalize(dir + random_dir_variation);
        vcl::vec3 new_pos = position + dt * (flock[i].speed * new_dir);

        flock[i].pos = new_pos;
        flock[i].dir = new_dir;

        mate_mesh.uniform.transform.translation = new_pos;
        mate_mesh.uniform.transform.rotation = rotation_between_vector_mat3(
            { 1, 0, 0 }, normalize(new_pos - old_pos));

        mate_mesh.uniform.color = { 0.2, 0.8, 1 };
        draw(mate_mesh, scene.camera);
    }
}

static void set_gui(timer_basic &timer)
{
    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale,
                        &scale_min, &scale_max, "%.2f s");

    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();
}

#endif