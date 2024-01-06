// Function to perform a single Runge-Kutta step
KineticState runge_kutta_step([[maybe_unused]] const KineticState& previous,
                              const KineticState& current)
{
    KineticState next;

    Eigen::Vector3f k1_v = current.acceleration;
    Eigen::Vector3f k1_x = current.velocity;
    Eigen::Vector3f k2_v = current.acceleration;
    Eigen::Vector3f k2_x = current.velocity + (time_step/2) * k1_v;
    Eigen::Vector3f k3_v = current.acceleration;
    Eigen::Vector3f k3_x = current.velocity + (time_step/2) * k2_v;
    Eigen::Vector3f k4_v = current.acceleration;
    Eigen::Vector3f k4_x = current.velocity + time_step * k3_v;

    next.velocity = current.velocity + (time_step/6) * (k1_v + 2*k2_v + 2*k3_v + k4_v);
    next.position = current.position + (time_step/6) * (k1_x + 2*k2_x + 2*k3_x + k4_x);
    next.acceleration = current.acceleration;
    return next;
}

// Function to perform a single Backward Euler step
KineticState backward_euler_step([[maybe_unused]] const KineticState& previous,
                                 const KineticState& current)
{
    KineticState next;
    next.position = current.position + (current.velocity + current.acceleration * time_step) * time_step;
    next.velocity = current.velocity + current.acceleration * time_step;
    next.acceleration = current.acceleration;
    return next;
}

// Function to perform a single Symplectic Euler step
KineticState symplectic_euler_step(const KineticState& previous, const KineticState& current)
{
    KineticState next;
    next.velocity = current.velocity + current.acceleration * time_step;
    next.position = current.position + next.velocity * time_step;
    next.acceleration = current.acceleration;
    (void)previous;
    return next;
}
