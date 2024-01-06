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
        if(object == this){
            continue;
        }
        // 检测该物体与另一物体是否碰撞的方法是：
        // 遍历该物体的每一条边，构造与边重合的射线去和另一物体求交，如果求交结果非空、
        // 相交处也在这条边的两个端点之间，那么该物体与另一物体发生碰撞。
        // 请时刻注意：物体 mesh 顶点的坐标都在模型坐标系下，你需要先将其变换到世界坐标系。
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            // v_indices 中是这条边两个端点的索引，以这两个索引为参数调用 GL::Mesh::vertex
            // 方法可以获得它们的坐标，进而用于构造射线。
            // 将模型坐标变换到世界坐标
            // 将模型坐标变换到世界坐标
            Vector3f p1 = (model() * mesh.vertex(v_indices[0]).homogeneous()).hnormalized();
            Vector3f p2 = (model() * mesh.vertex(v_indices[1]).homogeneous()).hnormalized();

            // 构造射线
            Ray ray{p1, (p2 - p1).normalized()};
            // 求交
            std::optional<Intersection> intersection;
            if (BVH_for_collision) {
                intersection =
                    object->bvh->intersect(ray, object->mesh, object->model());
            } else {
                intersection = naive_intersect(ray, object->mesh, object->model());
            }
            // 根据求交结果，判断该物体与另一物体是否发生了碰撞。
            if (intersection != std::nullopt &&
                (intersection->t * ray.direction).norm() <= (p2-p1).norm()) {
                // 如果发生碰撞，按动量定理计算两个物体碰撞后的速度，并将下一步状态的位置设为
                // current_state.position ，以避免重复碰撞。
                // 计算碰撞法向量
                //Vector3f normal = intersection.normal;
                // 计算碰撞后的速度
                float m1 = mass;
                float m2 = object->mass;
                Vector3f v1 = velocity;
                Vector3f v2 = object->velocity;
                Vector3f v1_prime = v1 - (2 * m2 / (m1 + m2)) *(v1 - v2);
                Vector3f v2_prime = v2 - (2 * m1 / (m1 + m2)) *(v2 - v1);
                // 更新速度
                next_state.velocity = v1_prime;
                object->velocity = v2_prime;
                // 更新位置
                center = current_state.position;
                //object->center = object->current_state.position;
                break;
            }
        }
    }
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
    prev_state = current_state;
    velocity = next_state.velocity;
    current_state.acceleration = next_state.acceleration;
}
std::optional<Intersection> naive_intersect(const Ray& ray, const GL::Mesh& mesh, const Matrix4f model)
{
    // 初始化一个空的交点结果
   Intersection result ;
    // 初始化一个无穷大的最小交点参数
    float closest_t = std::numeric_limits<float>::infinity();
    // 遍历网格的所有面片
    for (size_t i = 0; i < mesh.faces.count(); ++i) {
        // 获取面片的三个顶点
        Eigen::Vector3f a = mesh.vertex(mesh.face(i)[0]);
        Eigen::Vector3f b = mesh.vertex(mesh.face(i)[1]);
        Eigen::Vector3f c = mesh.vertex(mesh.face(i)[2]);
        // 对顶点进行模型变换
        a = (model * (mesh.vertex(mesh.face(i)[0])).homogeneous()).hnormalized();
        b = (model * (mesh.vertex(mesh.face(i)[1])).homogeneous()).hnormalized();
        c = (model * (mesh.vertex(mesh.face(i)[2])).homogeneous()).hnormalized();
        // 计算面片的法向量
        Eigen::Vector3f N = (b - a).cross(c - a);
        // 计算射线和平面的交点参数t
        float t = N.dot(a - ray.origin) / N.dot(ray.direction);
        // 检查t是否在有效范围内，交点在射线上
        if (t >= 0 && t < closest_t) {
            // 计算交点P
            Eigen::Vector3f P = ray.origin + t * ray.direction;
            // 计算三角形的面积
            float areaABC = 0.5f * N.norm();
            // 计算三个子三角形的面积
            float areaPBC = 0.5f * (b - P).cross(c - P).norm();
            float areaPAC = 0.5f * (a - P).cross(c - P).norm();
            float areaPAB = 0.5f * (a - P).cross(b - P).norm();
            // 计算重心坐标u, v, w
            float u = areaPBC / areaABC;
            float v = areaPAC / areaABC;
            float w = areaPAB / areaABC;
            // 判断点是否在三角形内部
            if (u >= 0 && u <= 1 && v >= 0 && v <= 1 && w >= 0 && w <= 1 && u + v + w <= 1 + eps) {
                // 更新最小交点参数
                closest_t = t;
                // 创建一个交点结果
                //result = Intersection{t, i, {u, v, w}, N.normalized()};
                closest_t                = t;
                result.t                 = t;
                result.face_index        = i;
                result.barycentric_coord = {u, v, w};
                result.normal            = N.normalized();
            }
        }
    }
    // 返回交点结果，如果没有交点，返回空的optional值
    return result;
}