#ifndef UGV_EKF1_HPP_
#define UGV_EKF1_HPP_

#include <Eigen/Dense>
using namespace Eigen;

using Vector5d = Eigen::Matrix<double, 5, 1>;

// this struct is a bridge to typical initialization
// from a ROS2 message source (like current pose)
struct ekf1Init {
    Vector3d x; // x, y, yaw
    Vector2d v_lr_mps;
    double t; // t0
};

// this struct generalizes what will come either from
// bag files (for testing) or robot serial comms (in the
// actual robot implementation).
struct ekf1Data {
    double gz_rps;
    Vector2d v_lr_mps;
    double tSec;
};

// state and twist estimates for output
// this could be ROS2-specific, but I'll allow
// this struct to conform to the EKF gadget
// herein.
struct ekf1Out {
    bool valid;
    Vector5d x;
    Matrix<double, 5, 5> P;
    Vector2d twist;
    Matrix<double, 2, 2> Pt;
};

inline void eigen_to_ros_cov(const Eigen::Matrix<double, 6, 6> &C, std::array<double, 36> &cov)
{
    using Mat6RM = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
    Eigen::Map<Mat6RM>(cov.data()) = C;
}

// Ekf1 (EKF1) is an extended Kalman filter which takes
// the robot wheel encoders (averaged to reduce noise) as
// wheel velocity estimates and improves integration of
// robot location by fusing robot orientation from the
// IMU gyroscope with a kinematic model whose wheel
// separation is in the EKF state space.
class Ekf1
{
  public:
    // these should be parameters loaded from a node config file
    double robot_dt;
    // u is [v_left, v_right] in meters/sec.
    Vector2d u_init;
    Vector2d u_prev;
    Matrix<double, 5, 1> x_init;
    Matrix<double, 5, 1> x_prev;
    double u_var;
    double t_prev;

    double var_b;
    double var_bg;
    double p_init;
    double var_imu_omega;
    const int n_dim = 5;
    const int m_dim = 2;
    Matrix<double, 5, 2> G;
    Matrix<double, 5, 5> P;
    Matrix<double, 5, 5> Q;
    Matrix<double, 5, 5> F;
    Matrix<double, 1, 5> H;
    Matrix<double, 2, 3> J;
    Matrix<double, 5, 5> I;
    Matrix<double, 2, 2> cov_u;
    Matrix<double, 3, 3> cov_u2;
    Matrix<double, 2, 2> Pt;
    double v;
    double omega;

    bool first_pass = true;

    Ekf1();

    void Init();

    void Init(const ekf1Init);

    // accept new data and observations. Note this particular EKF has synchronized
    // observations of state and data (so EKF predict and update are fused).
    ekf1Out Update(const ekf1Data update);

  private:

    // observation Jacobian
    Matrix<double, 1, 5> Hmat(const Vector5d& x, const Vector2d& u, const double dt);

    // error propagation matrix from u to process noise
    Matrix<double, 5, 2> Gmat(const Vector5d& x, const Vector2d& u, const double dt);

    // error propagation matrix from encoder v,omega to covariance v, omega
    Matrix<double, 2, 3> Jmat(const Vector5d& x, const Vector2d& u, const double dt);

    // state Jacobian
    Matrix<double, 5, 5> Fmat(const Vector5d& x, const Vector2d& u, const double dt);

    // state prediction
    Vector5d f(const Vector5d& x, const Vector2d& u, const double dt);

    // observation prediction
    double h(const Vector5d& x, const Vector2d& u, const double dt);
};

#endif  // UGV_EKF1_HPP_