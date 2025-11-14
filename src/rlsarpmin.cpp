#include "rlsarpmin.hpp"
#include <stdexcept>
#include <cmath>   // ★ 追加

namespace rlsarpmin {

// --- コンストラクタ ---
RLSARPMin::RLSARPMin(int p, int k, double lambda, double delta, double eps, int i)
    : p_(p), k_(k), lam_(lambda), eps_(eps),
      theta_(Eigen::VectorXd::Zero(p + 1)),
      P_(delta * Eigen::MatrixXd::Identity(p + 1, p + 1)),
      hist_(),
      index(i),
      y_last(0.0),
      result_last(0.0),
      y_vel(0.0),
      out_ma_buf_(),      // ← もう要らなければ削除してOK
      out_ma_sum_(0.0),   // ← 同上
      out_ema_(0.0),
      out_ema_init_(false),
      ema_alpha_(0.3)     // とりあえず 0.2〜0.5 あたりから試す
{
    if (p_ <= 0 || k_ <= 0) throw std::invalid_argument("p,k must be positive.");
    if (lam_ <= 0.0 || lam_ > 1.0) throw std::invalid_argument("lambda in (0,1].");
    if (delta <= 0.0) throw std::invalid_argument("delta must be positive.");
    if (eps_ < 0.0) throw std::invalid_argument("epsilon must be nonnegative.");
}

// --- 逐次更新（y_t を投入して k ステップ先予測を返す） ---
std::optional<double> RLSARPMin::operator()(double y_now, int k)
{
    // 角度 or 直線，どちらの差分を使うか
    if (index == 2) {
        y_vel = std::atan2(std::sin(y_now - y_last), std::cos(y_now - y_last));
    } else {
        y_vel = y_now - y_last;
    }
    y_last = y_now;

    // 履歴更新
    hist_.push_front(y_vel);
    if (static_cast<int>(hist_.size()) > p_ + 1) {
        hist_.pop_back();
    }

    // 履歴が足りないときは学習だけして終了
    if (static_cast<int>(hist_.size()) < p_ + 1) {
        return std::nullopt;
    }

    // x_t = [v_{t-1}, …, v_{t-p}, 1]^T
    Eigen::VectorXd x(p_ + 1);
    for (int i = 0; i < p_; ++i) {
        x(i) = hist_[i + 1];
    }
    x(p_) = 1.0;

    // 予測誤差 e_t = v_t - x^T θ_{t-1}
    const double e = hist_.front() - x.dot(theta_);

    // K_t = P x / (λ + x^T P x + ε)
    const Eigen::VectorXd v = P_ * x;
    const double d = lam_ + x.dot(v) + eps_;
    const Eigen::VectorXd K = v / d;

    // θ_t = θ_{t-1} + K e_t
    theta_ += K * e;

    // P_t = (P - K x^T P) / λ
    P_ = (P_ - K * (x.transpose() * P_)) / lam_;

    // 数値安定化
    P_ = 0.5 * (P_ + P_.transpose());
    P_.diagonal().array() += eps_;

    const int k_used = (k > 0 ? k : k_);

    // ★ 念のためガード：履歴不足時は0返し（本来ここに来ないはず）
    if (static_cast<int>(hist_.size()) < p_ + 1 || k_used <= 0) {
        return y_now;  // そのまま返す
    }

    const double result = kStepAhead_(k_used);

    // 異常ジャンプの検出
    if (std::abs(result - result_last) > 20.0) {
        result_last = result;   // ここも更新しておくかは好み
        return y_now;
    }
    result_last = result;

    // 未来 k ステップ先予測位置
    double y_pred = y_now + result;

    // ---- ここから指数移動平均（1次IIR） ----
    // ema_alpha_ は (0,1]．大きいほど追従性↑／小さいほど滑らか
    if (!out_ema_init_) {
        // 初回だけはそのまま採用
        out_ema_      = y_pred;
        out_ema_init_ = true;
    } else {
        out_ema_ = ema_alpha_ * y_pred + (1.0 - ema_alpha_) * out_ema_;
    }

    return out_ema_;
}

// --- 係数取得 ---
Eigen::VectorXd RLSARPMin::phi() const { return theta_.head(p_); }
double RLSARPMin::phi0() const { return theta_(p_); }

// --- 内部：k ステップ先予測 ---
double RLSARPMin::kStepAhead_(int k) const
{
    // ★ 防御的チェック：ここに来る時点でサイズは足りているはずだが，
    //   万一足りない場合に落ちないようにする
    if (static_cast<int>(hist_.size()) < p_ + 1 || k <= 0) {
        return 0.0;
    }

    Eigen::VectorXd h(p_);
    for (int i = 0; i < p_; ++i) {
        h(i) = hist_.at(i + 1);  // [v_{t-1}, ..., v_{t-p}]
    }

    const Eigen::VectorXd ar = theta_.head(p_);
    const double c = theta_(p_);

    double y_hat = hist_.front(); // v_t
    double y_sum = 0.0;           // 未来の増分 ∑_{j=1}^k v̂_{t+j}

    for (int step = 0; step < k; ++step) {
        y_hat = ar.dot(h) + c;
        y_sum += y_hat;

        // 履歴シフト
        for (int i = p_ - 1; i > 0; --i) {
            h(i) = h(i - 1);
        }
        h(0) = y_hat;
    }

    return y_sum;
}

} // namespace rlsarpmin
