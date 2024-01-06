Matrix4f Object::model()
{
    Matrix4f tran;
    tran << 1,0,0,center.x(),
                     0,1,0,center.y(),
                     0,0,1,center.z(),
                     0,0,0,1;
    const Quaternionf& r = rotation;
    auto [x_angle, y_angle, z_angle] = quaternion_to_ZYX_euler(r.w(), r.x(), r.y(), r.z());
    float &&x_curve = radians(x_angle);
    float &&y_curve = radians(y_angle);
    float &&z_curve = radians(z_angle);
    Matrix4f rotatx ;
    rotatx  <<         1,0,0,0,
                       0,cos(x_curve),-sin(x_curve),0,
                       0,sin(x_curve),cos(x_curve),0,
                       0,0,0,1;
    Matrix4f rotaty ;
    rotaty  << cos(y_curve),0,sin(y_curve),0,
               0,1,0,0,
               -sin(y_curve),0,cos(y_curve),0,
               0,0,0,1;
    Matrix4f rotatz ;
    rotatz    << cos(z_curve),-sin(z_curve),0,0,
                       sin(z_curve),cos(z_curve),0,0,
                       0,0,1,0,
                       0,0,0,1;             
    Matrix4f rotat;
    rotat = rotatx*rotaty*rotatz;
    Matrix4f scal ;
    scal << scaling.x(),0,0,0,
                      0,scaling.y(),0,0,
                      0,0,scaling.z(),0,
                      0,0,0,1;
    Matrix4f mod;
    mod  = tran * rotat * scal;
 
    return mod;
}