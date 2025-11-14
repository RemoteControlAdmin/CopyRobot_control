#pragma once
#include <Eigen/Dense>
#include <deque>
#include <optional>
#include <stdexcept>

namespace rlsarpmin {

class RLSARPMin {
public:
    // AR(p)+定数項，kステップ先予測
    // lambda∈(0,1]，delta≫1，epsilon≥0（数値安定用）
    RLSARPMin(int p, int k, double lambda = 0.99, double delta = 1e3, double eps = 1e-12, int i = 0);

    // 入力: y_t，出力: y_{t+k|t} or nullopt（履歴不足）
    std::optional<double> operator()(double y_now, int k = -1);

    Eigen::VectorXd phi()  const;
    double          phi0() const;

private:
    int p_, k_;
    int index;
    double lam_;
    double eps_;   // ← 追加：数値安定化ε
    Eigen::VectorXd  theta_;
    Eigen::MatrixXd  P_;
    std::deque<double> hist_;
    double y_last = 0;
    double result_last = 0;
    double y_vel;
    double kStepAhead_(int k) const;
    std::deque<double> out_ma_buf_;
    double out_ma_sum_ = 0.0;
    // rlsarpmin.hpp の private: などに
double out_ema_;        // 予測位置の指数移動平均
bool   out_ema_init_;   // 初期化済みかどうか
double ema_alpha_;      // フィルタ係数 (0,1]
};

} // namespace rlsarpmin
