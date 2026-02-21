#include <ugv/ekf1.hpp>
#include <iostream>

Ekf1::Ekf1() {
    robot_dt = 1.0/27;
    u_init = Vector2d{0.0, 0.0};
    x_init = Matrix<double, 5,1>{0, 0, 0, 0, 0.174*2};
    I.setZero();
    I.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0;
    u_var = 0.0001;
    cov_u.diagonal() << u_var, u_var;
    cov_u2.diagonal() << u_var, u_var, 0.0;
    var_b = 0.001;
    var_bg = 0.001;
    p_init = 0.001;
    var_imu_omega = 0.01;
    first_pass = true;
}

void Ekf1::Init() {
    first_pass = true;
    u_init = Vector2d{0.0, 0.0};
    x_init = Matrix<double, 5,1>{0, 0, 0, 0, 0.174*2};
    x_prev = x_init;
    u_prev = u_init;
    t_prev = 0.0;
    auto G = Gmat(x_init, u_init, robot_dt);
    P.diagonal() << p_init, p_init, p_init, var_bg*robot_dt, var_b*robot_dt;
    P = P + G * cov_u * G.transpose();
}

void Ekf1::Init(ekf1Init init_data) {
    first_pass = true;
    u_init = Vector2d{0.0, 0.0};
    x_init = Matrix<double, 5,1>{0, 0, 0, 0, 0.174*2};
    // update from input
    u_init = init_data.v_lr_mps;
    x_init.block<3,1>(0, 0) = init_data.x;
    x_prev = x_init;
    u_prev = u_init;
    t_prev = init_data.t;
    auto G = Gmat(x_init, u_init, robot_dt);
    P.setZero();
    P.diagonal() << p_init, p_init, p_init, var_bg*robot_dt, var_b*robot_dt;
    P = P + G * cov_u * G.transpose();
}

ekf1Out Ekf1::Update(const ekf1Data data){
    ekf1Out ekfOut;
    double t = data.tSec;
    double dt = t - t_prev;
    auto u = data.v_lr_mps;

    // predict
    auto x_pred = f(x_prev, u, dt);
    auto G = Gmat(x_prev, u, dt);
    Q = G * cov_u * G.transpose();
    Q(3,3) = var_bg * dt;
    Q(4,4) = var_b * dt;
    auto F = Fmat(x_pred, u, dt);
    P = F * P * F.transpose() + Q;
    // update

    // innovation
    double y = data.gz_rps - h(x_pred, u, dt);
    auto H = Hmat(x_pred, u, dt);
    double S = H * P * H.transpose() + var_imu_omega;
    auto K = P * H.transpose() / S;
    auto x = x_pred + K * y;
    P = (I - K * H) * P;
    
    // to twist (v, omega) for output
    Vector2d twist = Vector2d({
        (u(0) + u(1))/2.0,
        (u(1) - u(0)) / x(4)
    });

    // twist covariance
    J = Jmat(x_pred, u, dt);
    cov_u2(2,2) = P(4,4);
    Pt = J * cov_u2 * J.transpose();

    // package output
    ekfOut.valid = true;
    ekfOut.x = x;
    ekfOut.P = P;
    ekfOut.twist = twist;
    ekfOut.Pt = Pt;

    // shift data
    x_prev = x;
    t_prev = t;

    return ekfOut;
}

Matrix<double, 5, 5> Ekf1::Fmat(const Vector5d& x, const Vector2d& u, const double dt)
{
    Matrix<double, 5, 5> F;
    double v = (u(0) + u(1)) /2.0;
    double omega = (u(1) - u(0)) / x(4);
    F.setIdentity();
    F(0,2) = -dt * v * cos(x(2));
    F(1,2) =  dt * v * sin(x(2));
    F(2,4) = -dt * omega / x(4);
    return F;
}

Vector5d Ekf1::f(const Vector5d& x, const Vector2d& u, const double dt)
{
    double v = (u(0) + u(1)) /2.0;
    double omega = (u(1) - u(0)) / x(4);
    // Euler integration, for now
    Vector5d dx = Vector5d({
        dt * v * cos(x(2)),
        dt * v * sin(x(2)),
        dt * omega,
        dt * 0,
        dt * 0
    });

    return x + dx;
}

double Ekf1::h(const Vector5d& x, const Vector2d& u, const double dt)
{
    (void) dt;
    double omega = (u(1) - u(0)) / x(4);
    // add the gyro bias from the EKF state
    return omega + x(3);
}

Matrix<double, 5, 2> Ekf1::Gmat(const Vector5d& x, const Vector2d& u, const double dt)
{
    (void) u;
    double c = cos(x(2));
    double s = sin(x(2));
    Matrix<double, 5, 2> G;
    G << 0.5 * dt * c, 0.5 * dt * c,
         0.5 * dt * s, 0.5 * dt * s,
         -dt/x(4), dt/x(4),
         0, 0,
         0, 0 ;
    return G;
}

Matrix<double, 1, 5> Ekf1::Hmat(const Vector5d& x, const Vector2d& u, const double dt)
{
    (void) dt;
    Matrix<double, 1, 5> H = Matrix<double, 1, 5>({
        0,
        0,
        0,
        1.0,
        -(u(1)-u(0))/(x(4)*x(4))
    });
    return H;
}

Matrix<double, 2, 3> Ekf1::Jmat(const Vector5d& x, const Vector2d& u, const double dt)
{
    (void) dt;
    double b1 = 1.0 / x(4);
    Matrix<double, 2, 3> J;
    J << 0.5, 0.5, 0.0,
         b1, -b1, -(u(1)-u(0))/(x(4)*x(4));
    return J;
}

