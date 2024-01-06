void Toolbar::layout_mode(Scene& scene)
{
    if (ImGui::BeginTabItem("Layout")) {
        if (mode != WorkingMode::LAYOUT) {
            on_selection_canceled();
            mode = WorkingMode::LAYOUT;
        }
        scene_hierarchies(scene);

        Object* selected_object = scene.selected_object;
        if (selected_object != nullptr) {
            material_editor(selected_object->mesh.material);
            ImGui::SeparatorText("Transform");
            ImGui::Text("Translation");
            ImGui::PushID("Translation##");
            Vector3f& center = selected_object->center;
            xyz_drag(&center.x(), &center.y(), &center.z(), POSITION_UNIT);
            ImGui::PopID();

            ImGui::Text("Scaling");
            ImGui::PushID("Scaling##");
            Vector3f& scaling = selected_object->scaling;
            xyz_drag(&scaling.x(), &scaling.y(), &scaling.z(), SCALING_UNIT);
            ImGui::PopID();

            const Quaternionf& r             = selected_object->rotation;
            auto [x_angle, y_angle, z_angle] = quaternion_to_ZYX_euler(r.w(), r.x(), r.y(), r.z());
            ImGui::Text("Rotation (ZYX Euler)");
            ImGui::PushID("Rotation##");
            ImGui::PushItemWidth(0.3f * ImGui::CalcItemWidth());
            ImGui::DragFloat("pitch", &x_angle, ANGLE_UNIT, -180.0f, 180.0f, "%.1f deg",
                             ImGuiSliderFlags_AlwaysClamp);
            ImGui::SameLine();
            ImGui::DragFloat("yaw", &y_angle, ANGLE_UNIT, -90.0f, 90.0f, "%.1f deg",
                             ImGuiSliderFlags_AlwaysClamp);
            ImGui::SameLine();
            ImGui::DragFloat("roll", &z_angle, ANGLE_UNIT, -180.0f, 180.0f, "%.1f deg",
                             ImGuiSliderFlags_AlwaysClamp);
            ImGui::PopItemWidth();
            ImGui::PopID();
            float sin_half_x_angle = sin(radians(x_angle)*0.5);
            float sin_half_y_angle = sin(radians(y_angle)*0.5);
            float sin_half_z_angle = sin(radians(z_angle)*0.5);
            float cos_half_x_angle = cos(radians(x_angle)*0.5);
            float cos_half_y_angle = cos(radians(y_angle)*0.5);
            float cos_half_z_angle = cos(radians(z_angle)*0.5);
            Eigen::Quaternionf quaternionx,quaterniony,quaternionz;
            quaternionx.x() =  Vector3f::UnitX().x() * sin_half_x_angle;
            quaternionx.y() =  Vector3f::UnitX().y() * sin_half_x_angle;
            quaternionx.z() =  Vector3f::UnitX().z() * sin_half_x_angle;
            quaternionx.w() =  cos_half_x_angle;
            quaterniony.x() =  Vector3f::UnitY().x() * sin_half_y_angle;
            quaterniony.y() =  Vector3f::UnitY().y() * sin_half_y_angle;
            quaterniony.z() =  Vector3f::UnitY().z() * sin_half_y_angle;
            quaterniony.w() =  cos_half_y_angle;
            quaternionz.x() =  Vector3f::UnitZ().x() * sin_half_z_angle;
            quaternionz.y() =  Vector3f::UnitZ().y() * sin_half_z_angle;
            quaternionz.z() =  Vector3f::UnitZ().z() * sin_half_z_angle;
            quaternionz.w() =  cos_half_z_angle;

            selected_object->rotation = quaternionx*quaterniony*quaternionz;
            
            }
        ImGui::EndTabItem();
    }
}