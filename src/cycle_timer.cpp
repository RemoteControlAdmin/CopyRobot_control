#include "cycle_timer.hpp"
#include <thread>  // sleep_until, yield
#include <cassert>

namespace cycle_timer {

CycleTimer::CycleTimer(Duration period, Duration spin_margin) noexcept
    : T_(period),
      spin_(spin_margin),
      next_(Clock::now() + period),
      last_wake_(Clock::now())
{
    // 不変条件（assertはReleaseで無効になるので必要なら実行時チェックに）
    assert(T_   > Duration::zero());
    assert(spin_ >= Duration::zero());
    assert(spin_ <= T_);
}

CycleTimer::Duration CycleTimer::tick() noexcept {
    const auto deadline = next_;
    const auto coarse   = deadline - spin_;

    if (const auto now = Clock::now(); now < coarse) {
        std::this_thread::sleep_until(coarse);
    }
    while (Clock::now() < deadline) {
        std::this_thread::yield(); // 好みで削除可
    }

    const auto wake = Clock::now();
    const auto dt   = wake - last_wake_;
    last_wake_ = wake;

    // 次回締切を進め，過走があれば“まとめて”追いつく
    next_ += T_;
    const auto now2 = Clock::now();
    if (now2 > next_) {
        const auto lag = (now2 - next_) / T_; // 0以上の整数
        next_ += (lag + 1) * T_;
    }
    return dt;
}

} // namespace cycle_timer
