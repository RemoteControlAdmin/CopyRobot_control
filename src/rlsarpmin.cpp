#include "rlsarpmin.hpp"
#include <stdexcept>

namespace rlsarpmin {

// --- コンストラクタ ---
RLSARPMin::RLSARPMin(int p, int k, double lambda, double delta, double eps, int i)
    : p_(p), k_(k), lam_(lambda), eps_(eps),
      theta_(Eigen::VectorXd::Zero(p + 1)),                 // [φ1..φp, φ0]^T
      P_(delta * Eigen::MatrixXd::Identity(p + 1, p + 1)),  // 共分散
      hist_(),
      index(i)
{
    if (p_ <= 0 || k_ <= 0) throw std::invalid_argument("p,k must be positive.");
    if (lam_ <= 0.0 || lam_ > 1.0) throw std::invalid_argument("lambda in (0,1].");
    if (delta <= 0.0) throw std::invalid_argument("delta must be positive.");
    if (eps_ < 0.0) throw std::invalid_argument("epsilon must be nonnegative.");
}

// --- 逐次更新（y_t を投入して k ステップ先予測を返す） ---
std::optional<double> RLSARPMin::operator()(double y_now, int k)
{
    if(index ==2){
        y_vel = atan2(sin(y_now - y_last), cos(y_now - y_last));
    }
    y_vel = y_now - y_last;
    y_last = y_now;
    // 履歴更新
    hist_.push_front(y_vel);
    if (static_cast<int>(hist_.size()) > p_ + 1) hist_.pop_back();

    if (static_cast<int>(hist_.size()) < p_ + 1) return std::nullopt;

    // x_t = [y_{t-1}, …, y_{t-p}, 1]^T
    Eigen::VectorXd x(p_ + 1);
    for (int i = 0; i < p_; ++i) x(i) = hist_[i + 1];
    x(p_) = 1.0;

    // 予測誤差 e_t = y_t - x^T θ_{t-1}
    const double e = hist_.front() - x.dot(theta_);

    // K_t = P x / (λ + x^T P x + ε)
    const Eigen::VectorXd v = P_ * x;
    const double d = lam_ + x.dot(v) + eps_;  // ← εを加算
    const Eigen::VectorXd K = v / d;

    // θ_t = θ_{t-1} + K e_t
    theta_ += K * e;

    // P_t = (P - K x^T P) / λ
    P_ = (P_ - K * (x.transpose() * P_)) / lam_;

    // 数値安定化：Pを対称化し，対角にεを加算
    P_ = 0.5 * (P_ + P_.transpose());
    P_.diagonal().array() += eps_;

    const int k_used = (k > 0 ? k : k_);
    auto result = kStepAhead_(k_used);
    if (abs(result - result_last) > 20){
        return y_now;
    }
    result_last = result;
    result += y_now;
    return result;
}

// --- 係数取得 ---
Eigen::VectorXd RLSARPMin::phi() const { return theta_.head(p_); }
double RLSARPMin::phi0() const { return theta_(p_); }

// --- 内部：k ステップ先予測 ---
double RLSARPMin::kStepAhead_(int k) const
{
    Eigen::VectorXd h(p_);
    for (int i = 0; i < p_; ++i) h(i) = hist_.at(i + 1);

    const Eigen::VectorXd ar = theta_.head(p_);
    const double c = theta_(p_);
    double y_hat = hist_.front();
    double y_sam = 0;
    for (int step = 0; step < k; ++step) {
        y_hat = ar.dot(h) + c;
        for (int i = p_ - 1; i > 0; --i) h(i) = h(i - 1);
        h(0) = y_hat;
        y_sam += y_hat;
    }
    return y_sam;
}

} // namespace rlsarpmin
