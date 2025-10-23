#pragma once
#include <chrono>  // ← ヘッダではchronoだけでOK（<thread>は不要）

namespace cycle_timer {

class CycleTimer {
public:
    using Clock    = std::chrono::steady_clock;
    using Duration = Clock::duration;

    // 期間とスピン余裕（デフォルト 150us）
    explicit CycleTimer(Duration period,
                        Duration spin_margin = std::chrono::microseconds(150)) noexcept;

    // 1 サイクル進めて，“休んだ後の周期”を返す
    [[nodiscard]] Duration tick() noexcept;

    // （必要ならAPIをここに追加）

private:
    Duration T_;
    Duration spin_;
    Clock::time_point next_;
    Clock::time_point last_wake_;
};

} // namespace cycle_timer
