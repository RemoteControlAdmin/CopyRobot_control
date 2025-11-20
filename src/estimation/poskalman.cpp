#include "estimation/poskalman.hpp"
#include <cmath>

PoseKalmanFilter::PoseKalmanFilter()
{
    x_.setZero();

    // 初期共分散（適当な例）
    P_.setIdentity();
    P_ *= 1e-3;

    // 状態遷移（predict のたびに dt で更新）
    F_.setIdentity();

    // 観測行列（速度観測も含めるので I_6）
    H_.setIdentity();

    // プロセスノイズ Q 初期値
    setProcessNoise(
        1e-4, 1e-4, 1e-3, 1e-2, 1e-2, 1e-2
    );

    // 観測ノイズ R 初期値（速度側はやや大きめ）
    setMeasurementNoise(
        2e-3, 2e-3, 2e-4,
        1e-1, 1e-1, 1e-2
    );
}

void PoseKalmanFilter::setInitialState(double x,
                                       double y,
                                       double theta,
                                       double vx,
                                       double vy,
                                       double omega)
{
    x_(0) = x;
    x_(1) = y;
    x_(2) = wrapAngle(theta);
    x_(3) = vx;
    x_(4) = vy;
    x_(5) = omega;
}

void PoseKalmanFilter::setInitialCovariance(double var_x,
                                            double var_y,
                                            double var_theta,
                                            double var_vx,
                                            double var_vy,
                                            double var_omega)
{
    P_.setZero();
    P_(0,0) = var_x;
    P_(1,1) = var_y;
    P_(2,2) = var_theta;
    P_(3,3) = var_vx;
    P_(4,4) = var_vy;
    P_(5,5) = var_omega;
}

void PoseKalmanFilter::setProcessNoise(double q_x,
                                       double q_y,
                                       double q_theta,
                                       double q_vx,
                                       double q_vy,
                                       double q_omega)
{
    Q_.setZero();
    Q_(0,0) = q_x;
    Q_(1,1) = q_y;
    Q_(2,2) = q_theta;
    Q_(3,3) = q_vx;
    Q_(4,4) = q_vy;
    Q_(5,5) = q_omega;
}

void PoseKalmanFilter::setMeasurementNoise(double r_x,
                                           double r_y,
                                           double r_theta,
                                           double r_vx,
                                           double r_vy,
                                           double r_omega)
{
    R_.setZero();
    R_(0,0) = r_x;
    R_(1,1) = r_y;
    R_(2,2) = r_theta;
    R_(3,3) = r_vx;
    R_(4,4) = r_vy;
    R_(5,5) = r_omega;
}

void PoseKalmanFilter::updateTransitionMatrix(double dt)
{
    F_.setIdentity();
    F_(0,3) = dt;
    F_(1,4) = dt;
    F_(2,5) = dt;
}

void PoseKalmanFilter::predict(double dt)
{
    updateTransitionMatrix(dt);

    x_ = F_ * x_;
    x_(2) = wrapAngle(x_(2));

    P_ = F_ * P_ * F_.transpose() + Q_;
}

void PoseKalmanFilter::update(double meas_x,
                              double meas_y,
                              double meas_theta,
                              double meas_vx,
                              double meas_vy,
                              double meas_omega)
{
    MeasVec z;
    z << meas_x,
         meas_y,
         wrapAngle(meas_theta),
         meas_vx,
         meas_vy,
         meas_omega;

    MeasVec z_pred = H_ * x_;
    z_pred(2) = wrapAngle(z_pred(2));

    MeasVec y = z - z_pred;
    y(2) = wrapAngle(y(2));

    MeasCov S = H_ * P_ * H_.transpose() + R_;

    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K =
        P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    x_(2) = wrapAngle(x_(2));

    StateMat I = StateMat::Identity();
    P_ = (I - K * H_) * P_;
}

double PoseKalmanFilter::wrapAngle(double angle)
{
    const double two_pi = 2.0 * M_PI;
    angle = std::fmod(angle + M_PI, two_pi);
    if (angle < 0.0) angle += two_pi;
    return angle - M_PI;
}
