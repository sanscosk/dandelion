BVHNode* BVH::recursively_build(vector<size_t> faces_idx)
{
    BVHNode* node = new BVHNode();

    AABB aabb;
    for (size_t i = 0; i < faces_idx.size(); i++) {
        aabb = union_AABB(aabb, get_aabb(mesh, faces_idx[i]));
    }
    // 如果只有一个面，创建叶子节点
    if (faces_idx.size() == 1) {
        node->aabb = aabb;
        node->face_idx = faces_idx[0];
        return node;
    }
    // 如果有两个面，创建两个叶子节点
    if (faces_idx.size() == 2) {
        node->left = recursively_build({faces_idx[0]});
        node->right = recursively_build({faces_idx[1]});
        node->aabb = union_AABB(node->left->aabb, node->right->aabb);
        return node;
    }
    // 如果有多个面，选择最长的维度进行划分
    else {
        // 选择最长的维度
        int axis = aabb.max_extent(); 
        // 根据中间点划分面
        vector<size_t> left_faces, right_faces;
        // 对面的索引按照中心坐标的分量排序
        sort(
            faces_idx.begin(), faces_idx.end(), [this,axis](size_t a, size_t b){
                // 使用get_aabb函数来获取面的包围盒，然后使用centroid函数来获取面的中心坐标
                Eigen::Vector3f center_a = get_aabb(this->mesh, a).centroid();
                Eigen::Vector3f center_b = get_aabb(this->mesh, b).centroid();
                return center_a[axis] < center_b[axis];
            });
        // 取排序后的中间索引作为划分点
        size_t mid = faces_idx.size() / 2;
        // 将前半部分的面划分到左子树，后半部分的面划分到右子树
        for (size_t i = 0; i < faces_idx.size(); i++) {
            if (i < mid) {
                left_faces.push_back(faces_idx[i]);
            }
            else {
                right_faces.push_back(faces_idx[i]);
            }
        }
        // 递归建立左右子树
        node->left = recursively_build(left_faces);
        node->right = recursively_build(right_faces);
        // 合并左右子树的包围盒
        node->aabb = union_AABB(node->left->aabb, node->right->aabb);
        return node;
    }
}
// 发射的射线与当前节点求交，并递归获取最终的求交结果
optional<Intersection> BVH::ray_node_intersect(BVHNode* node, const Ray& ray) const
{
    optional<Intersection> isect;
    // The node intersection is performed in the model coordinate system.
    // Therefore, the ray needs to be transformed into the model coordinate system.
    // The intersection attributes returned are all in the model coordinate system.
    // Therefore, They are need to be converted to the world coordinate system.    
    // If the model shrinks, the value of t will also change.
    // The change of t can be solved by intersection point changing simultaneously

    // transform ray to model coordinate system
    Ray model_ray = ray;

    Matrix4f world_to_model = model.inverse();
    Vector3f model_origin = (world_to_model * (Vector4f() << ray.origin, 1.0f).finished()).head<3>();
    Vector3f model_target = (world_to_model * (Vector4f() << (ray.origin + ray.direction), 1.0f).finished()).head<3>();
    model_ray.origin = model_origin;
    model_ray.direction = (model_target - model_origin).normalized();

    // 计算射线方向的倒数
    Vector3f inv_dir(1 / model_ray.direction.x(), 1 / model_ray.direction.y(),
                     1 / model_ray.direction.z());

    // 判断射线在各个坐标轴上的正负方向，1为正向，0为负向
    std::array<int, 3> dir_is_neg = {(model_ray.direction.x() <= 0) ? 0 : 1,
                                     (model_ray.direction.y() <= 0) ? 0 : 1,
                                     (model_ray.direction.z() <= 0) ? 0 : 1};

    // 检查射线是否与当前节点的包围盒相交
    if (!node->aabb.intersect(model_ray, inv_dir, dir_is_neg)) {
        // 如果不相交，直接返回空值
        return isect;
    }

    // 如果当前节点是叶子节点，则求解射线与三角形的交点
    if (!node->left && !node->right) {
        optional<Intersection> result = ray_triangle_intersect(model_ray, mesh, node->face_idx);
        if (!result.has_value()) {
            return std::nullopt;
        }
        // 将交点属性从模型坐标系转换到世界坐标系
        Intersection result_world = *result;
        result_world.normal = (model.inverse().transpose() * Eigen::Vector4f(result_world.normal.x(), result_world.normal.y(), result_world.normal.z(), 0.0f)).head<3>().normalized();
        return result_world;
    }

    // 如果当前节点不是叶子节点，则递归地在左右子节点上进行射线求交
    optional<Intersection> left_result = ray_node_intersect(node->left, ray);
    optional<Intersection> right_result = ray_node_intersect(node->right, ray);

    // 返回最近的交点
    if (left_result.has_value() && (!right_result.has_value() || left_result->t < right_result->t)) {
        return left_result;
    }
    return right_result;

}
// 判断当前射线是否与当前AABB相交
bool AABB::intersect(const Ray& ray, const Vector3f& inv_dir, const std::array<int, 3>& dir_is_neg)
{ 
  //射线沿着对应的轴与包围盒的两个平面的相交距离
  Vector3f imin;
  imin.x() = (p_min.x() -  ray.origin.x()) * inv_dir.x();
  imin.y() = (p_min.y() -  ray.origin.y()) * inv_dir.y();
  imin.z() = (p_min.z() -  ray.origin.z()) * inv_dir.z();
  Vector3f imax;
  imax.x() = (p_max.x() -  ray.origin.x()) * inv_dir.x();
  imax.y() = (p_max.y() -  ray.origin.y()) * inv_dir.y();
  imax.z() = (p_max.z() -  ray.origin.z()) * inv_dir.z();

  //imin的每个分量都小于或等于imax的对应分量
  if(dir_is_neg[0]<0){ std::swap(imin.x(),imax.x()); }    
  if(dir_is_neg[1]<0){ std::swap(imin.y(),imax.y()); }
  if(dir_is_neg[2]<0){ std::swap(imin.z(),imax.z()); }

  //找出imin的最大分量和imax的最小分量
  float n = std::max(imin.x(),std::max(imin.y(),imin.z()));
  float f = std::min(imax.x(),std::min(imax.y(),imax.z()));
  
  //射线与包围盒有相交
  return f>=n&&n>0;

}
optional<Intersection> ray_triangle_intersect(const Ray& ray, const GL::Mesh& mesh, size_t index)
{
 // 对顶点进行模型变换
    Eigen::Vector3f a = mesh.vertex(mesh.face(index)[0]);
    Eigen::Vector3f b = mesh.vertex(mesh.face(index)[1]);
    Eigen::Vector3f c = mesh.vertex(mesh.face(index)[2]);

    // 计算面片的法向量
    Eigen::Vector3f N = (b - a).cross(c - a);

    // 计算射线和平面的交点参数t
    float t = N.dot(a - ray.origin) / N.dot(ray.direction);
    if (t < 0 || N.dot(ray.direction) == 0) {
        return std::nullopt; // 交点在射线起点之前或射线与平面平行
    }

    // 计算交点P
    Eigen::Vector3f P = ray.origin + t * ray.direction;

    // 计算三角形的面积
    float areaABC = N.norm() / 2.0f;

    // 计算三个子三角形的面积
    float areaPBC = (b - P).cross(c - P).norm() / 2.0f;
    float areaPAC = (a - P).cross(c - P).norm() / 2.0f;
    float areaPAB = (a - P).cross(b - P).norm() / 2.0f;

    // 计算重心坐标u, v, w
    float u = areaPBC / areaABC;
    float v = areaPAC / areaABC;
    float w = areaPAB / areaABC;

    // 判断点是否在三角形内部
    if (u >= 0 && u <= 1 && v >= 0 && v <= 1 && w >= 0 && w <= 1 && u + v + w <= 1 + eps) {
        // 创建一个交点结果
        Intersection result;
        result.t = t;
        result.face_index = index;
        result.barycentric_coord = {u, v, w};
        result.normal = N.normalized();
        return result;
    }

    // 如果不在三角形内部，返回空的optional值
    return std::nullopt;
}