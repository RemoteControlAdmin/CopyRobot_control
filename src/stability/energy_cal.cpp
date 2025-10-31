
# include "stability/energy_cal.hpp"

namespace stability_lib{
    EnergyCal::EnergyCal(){} //コンストラクタ
    
    double EnergyCal::loc_energy_cal(std::vector<double> copy_data, 
        std::vector<double> partner_master_data, std::vector<double> force_actual_data){ 

        double copy_power = (copy_data[3] *force_actual_data[0]*cos(force_actual_data[1])) + 
                            (copy_data[4] *force_actual_data[0]*sin(force_actual_data[1]));
        double partner_master_power = (partner_master_data[3] *force_actual_data[0]*cos(force_actual_data[1]) + M_PI) +
                                     (partner_master_data[4] *force_actual_data[0]*sin(force_actual_data[1]) +  M_PI);
        double loc_energy = copy_power + partner_master_power;

        return loc_energy;
    }

    double EnergyCal::sum_energy_cal(RobotData robotdata, int64_t send_time){
        double remote_energy = loc_energy_cal(robotdata.remote_copy_data,
            robotdata.master_data, robotdata.force_udp_data);

        double local_energy = nearest(send_time);

        sum_energy += (remote_energy + local_energy);

        local_energy = loc_energy_cal(robotdata.copy_data,
            robotdata.partner_master_data, robotdata.force_actual_data);
        push(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count(), local_energy);

        return sum_energy;
    }

    void EnergyCal::push(int64_t t_ns, double x){
        if (q_.size() == K) q_.pop_front();
        q_.push_back({t_ns, x});
    }

    // 最近傍（線形）
    double EnergyCal::nearest(int64_t t)  {
        const std::size_t n = q_.size();
        if (n == 0) return 0.0;
        if (n == 1) return q_.front().x;

        // 1) 範囲外：前側（t <= t0） → [0,1] で外挿
        if (t <= q_.front().t_ns) {
            const Rec& a = q_.front();
            const Rec& b = q_[1];
            const std::int64_t dt = b.t_ns - a.t_ns;
            if (dt == 0) return a.x;
            const double alpha = static_cast<double>(t - a.t_ns) / static_cast<double>(dt);
            return a.x + alpha * (b.x - a.x);
        }

        // 2) 範囲外：後側（t >= t_{N-1}） → [N-2,N-1] で外挿
        if (t >= q_.back().t_ns) {
            const Rec& b = q_.back();
            const Rec& a = q_[n - 2];
            const std::int64_t dt = b.t_ns - a.t_ns;
            if (dt == 0) return b.x;
            const double alpha = static_cast<double>(t - b.t_ns) / static_cast<double>(dt);
            return b.x + alpha * (b.x - a.x); // = b.x + v*(t-b.t_ns)
        }

        // 3) 範囲内：挟み込む区間 [i-1, i] を探して補間
        for (std::size_t i = 1; i < n; ++i) {
            const Rec& a = q_[i - 1];
            const Rec& b = q_[i];
            if (a.t_ns <= t && t <= b.t_ns) {
                const std::int64_t dt = b.t_ns - a.t_ns;
                if (dt == 0) return a.x; // 同一時刻が連続する異常ケースの保険
                const double alpha = static_cast<double>(t - a.t_ns) / static_cast<double>(dt);
                return a.x + alpha * (b.x - a.x);
            }
        }

        // ここには来ないはず（上で全ケースを返している）
        return 0.0;
    }
    
    EnergyCal::~EnergyCal() {} //　デストラクタ
}
