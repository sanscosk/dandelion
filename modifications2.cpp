Matrix4f Camera::projection()
{
    const float fov_y = radians(fov_y_degrees);
    const float top   = abs(near) * std::tan(fov_y / 2.0f);
    const float bottom = -top;
    const float right = top * aspect_ratio;
    const float left = -right;
    
    Matrix4f projection;
    Matrix4f Mortho;
    Mortho << 2/(right-left),0,0,(left+right)/(right-left),
              0,2/(top-bottom),0,(bottom+top)/(top-bottom),
              0,0,2/(near-far),-(near+far)/(near-far),
              0,0,0,-1;
    Matrix4f Mper2ortho;
    Mper2ortho << near,0,0,0,
                  0,near,0,0,
                  0,0,near+far,near*far,
                  0,0,1,0;
    projection = Mortho * Mper2ortho;
    
    return projection;
}