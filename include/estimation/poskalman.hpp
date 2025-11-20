#pragma once

#include <Eigen/Dense>

/**
 * @brief 2D姿勢 (x, y, theta) とその速度 (vx, vy, omega) を扱うカルマンフィルタ．
 *
 * 状態ベクトル:
 *   x = [ x, y, theta, vx, vy, omega ]^T
 *
 * 観測ベクトル:
 *   y = [ x_obs, y_obs, theta_obs, vx_obs, vy_obs, omega_obs ]^T
 *
 * 状態方程式:
 *   x_k = F_k x_{k-1} + w_k
 *   （F_k は dt 依存の定速度モデル）
 *
 * 観測方程式:
 *   y_k = H x_k + v_k（H = I_6）
 */
class PoseKalmanFilter {
public:
    static constexpr int STATE_DIM = 6;
    static constexpr int MEAS_DIM  = 6;

    using StateVec = Eigen::Matrix<double, STATE_DIM, 1>;
    using StateMat = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
    using MeasVec  = Eigen::Matrix<double, MEAS_DIM, 1>;
    using MeasMat  = Eigen::Matrix<double, MEAS_DIM, STATE_DIM>;
    using MeasCov  = Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>;

    /// コンストラクタ（行列の初期化）
    PoseKalmanFilter();

    /// 初期状態設定
    void setInitialState(double x,
                         double y,
                         double theta,
                         double vx = 0.0,
                         double vy = 0.0,
                         double omega = 0.0);

    /// 初期共分散設定
    void setInitialCovariance(double var_x,
                              double var_y,
                              double var_theta,
                              double var_vx,
                              double var_vy,
                              double var_omega);

    /// プロセスノイズ Q 設定
    void setProcessNoise(double q_x,
                         double q_y,
                         double q_theta,
                         double q_vx,
                         double q_vy,
                         double q_omega);

    /// 観測ノイズ R 設定
    void setMeasurementNoise(double r_x,
                             double r_y,
                             double r_theta,
                             double r_vx,
                             double r_vy,
                             double r_omega);

    /// 予測ステップ（dt は観測タイムスタンプ差）
    void predict(double dt);

    /// 更新ステップ（速度観測あり）
    void update(double meas_x,
                double meas_y,
                double meas_theta,
                double meas_vx,
                double meas_vy,
                double meas_omega);

    /// 状態取得
    StateVec getState() const;
    StateMat getCovariance() const;

    /// 個別ゲッタ
    double getX() const;
    double getY() const;
    double getTheta() const;
    double getVx() const;
    double getVy() const;
    double getOmega() const;

private:
    /// 角度を [-pi, pi) にラップする
    static double wrapAngle(double angle);

    /// 状態遷移行列 F を dt で更新
    void updateTransitionMatrix(double dt);

private:
    StateVec x_;   ///< 状態
    StateMat P_;   ///< 共分散
    StateMat F_;   ///< 状態遷移行列
    MeasMat  H_;   ///< 観測行列（常に I_6）
    StateMat Q_;   ///< プロセスノイズ
    MeasCov  R_;   ///< 観測ノイズ
};

// ------------ inline 部分 ------------

inline PoseKalmanFilter::StateVec PoseKalmanFilter::getState() const {
    return x_;
}

inline PoseKalmanFilter::StateMat PoseKalmanFilter::getCovariance() const {
    return P_;
}

inline double PoseKalmanFilter::getX() const     { return x_(0); }
inline double PoseKalmanFilter::getY() const     { return x_(1); }
inline double PoseKalmanFilter::getTheta() const { return x_(2); }
inline double PoseKalmanFilter::getVx() const    { return x_(3); }
inline double PoseKalmanFilter::getVy() const    { return x_(4); }
inline double PoseKalmanFilter::getOmega() const { return x_(5); }
