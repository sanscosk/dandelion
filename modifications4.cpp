void Scene::simulation_update()
{
    // 这次模拟的总时长不是上一帧的时长，而是上一帧时长与之前帧剩余时长的总和，
    time_point t_now = steady_clock::now();
    // 即上次调用 simulation_update 到现在过了多久。
    duration frame_duration    = t_now - last_update;
    duration remained_duration = frame_duration;
    // time_step ，当总时长不够一个 time_step 时停止模拟。
    while (remained_duration.count() >= time_step) {
        for (auto& group : groups) {
            for (auto& object : group->objects) {
                object->update(all_objects);
            }
        }
        // 以固定的时间步长 (time_step) 循环模拟物体运动，每模拟一步，模拟总时长就减去一个
        // auto timestep = std::chrono::duration_cast(time_step);
        std::chrono::duration<float> timestep(time_step);
        double time_step_db = 1.0 / 30.0;
        std::chrono::duration<double> time_step_duration(time_step_db);
        auto time_step_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(time_step_duration);
        // std::chrono::duration<double> time_interval(1.0 / 30.0);
        // std::chrono::seconds timestepp(1);
        // timestepp *= time_step;
        remained_duration = remained_duration - timestep;
        // 根据刚才模拟时间步的数量，更新最后一次调用 simulation_update 的时间 (last_update)
        // auto last_duration = last_update.time_since_epoch();
        // std::chrono::duration<double> time_interval(1.0 / 30.0);
        // auto new_duration = last_duration + time_interval;
        // std::chrono::time_point<std::chrono::steady_clock> new_time_point(new_duration);
        // last_update = new_time_point;
        last_update += time_step_ns;
    }
}


void Object::update(vector<Object*>& all_objects)
{
    // 首先调用 step 函数计下一步该物体的运动学状态。
    KineticState current_state{center, velocity, force / mass};
    KineticState next_state = step(prev_state, current_state);
    (void)next_state;
    // 将物体的位置移动到下一步状态处，但暂时不要修改物体的速度。
    center = next_state.position;
    // 遍历 all_objects，检查该物体在下一步状态的位置处是否会与其他物体发生碰撞。
    for (auto object : all_objects) {
        (void)object;

        // 检测该物体与另一物体是否碰撞的方法是：
        // 遍历该物体的每一条边，构造与边重合的射线去和另一物体求交，如果求交结果非空、
        // 相交处也在这条边的两个端点之间，那么该物体与另一物体发生碰撞。
        // 请时刻注意：物体 mesh 顶点的坐标都在模型坐标系下，你需要先将其变换到世界坐标系。
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            (void)v_indices;
            // v_indices 中是这条边两个端点的索引，以这两个索引为参数调用 GL::Mesh::vertex
            // 方法可以获得它们的坐标，进而用于构造射线。
            if (BVH_for_collision) {
            } else {
            }
            // 根据求交结果，判断该物体与另一物体是否发生了碰撞。
            // 如果发生碰撞，按动量定理计算两个物体碰撞后的速度，并将下一步状态的位置设为
            // current_state.position ，以避免重复碰撞。
        }
    }
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
    prev_state = current_state;
    velocity = next_state.velocity;
    current_state.acceleration = next_state.acceleration;


}


// Function to perform a single Forward Euler step
KineticState forward_euler_step([[maybe_unused]] const KineticState& previous,
                                const KineticState& current)
{
    KineticState next;
    next.position = current.position + current.velocity * time_step;
    next.velocity = current.velocity + current.acceleration * time_step;
    next.acceleration = current.acceleration;
    return next;
}