
# include "stability/energy_cal.hpp"

namespace stability_lib{
    EnergyCal::EnergyCal(){
        sum_energy = 0.0;
    } //コンストラクタ
    
    double EnergyCal::loc_energy_cal(std::vector<double> copy_data, 
        std::vector<double> master_data, std::vector<double> force_actual_data){ 
        double angle = atan2((master_data[1]-copy_data[1]),(master_data[0]-copy_data[0]));
        double copy_power = (copy_data[3] *force_actual_data[0]*cos(angle + M_PI)) + 
                            (copy_data[4] *force_actual_data[0]*sin(angle + M_PI));
        double master_power = (master_data[3] *force_actual_data[0]*cos(angle)) +
                                     (master_data[4] *force_actual_data[0]*sin(angle));
        double loc_energy = copy_power + master_power;

        return loc_energy;
    }

    double EnergyCal::sum_energy_cal(RobotData robotdata, int64_t send_time){
        double remote_energy = loc_energy_cal(robotdata.remote_copy_data,
            robotdata.master_data, robotdata.force_udp_data);
        double local_energy = loc_energy_cal(robotdata.copy_data,
            robotdata.partner_master_data, robotdata.force_actual_data);
        push(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count(), local_energy);
        local_energy = nearest(send_time);
        if (robotdata.force_virtual_data[2] > robot_d){
            local_energy = 0.0;
            remote_energy = 0.0;
        }
        sum_energy += (remote_energy + local_energy);

        return sum_energy;
    }

    void EnergyCal::push(int64_t t_ns, double x){
        if (q_.size() == K) q_.pop_front();
        q_.push_back({t_ns, x});
    }

    // 最近傍（線形）
    double EnergyCal::nearest(int64_t t) {
        if (q_.empty()) return 0.0;

        // t_ns が昇順に並んでいる前提（push 時に単調増加）
        auto it = std::lower_bound(
            q_.begin(), q_.end(), t,
            [](const Rec& r, int64_t val){ return r.t_ns < val; }
        );

        if (it == q_.begin()) {
            // すべての記録が t 以上 → 先頭が最も近い
            return it->x;
        }
        if (it == q_.end()) {
            // すべての記録が t 未満 → 末尾が最も近い
            return q_.back().x;
        }

        // 挟み込み：*(it-1) と *it のどちらが t に近いか
        const Rec& b = *it;
        const Rec& a = *(it - 1);

        const auto da = (t >= a.t_ns) ? (t - a.t_ns) : (a.t_ns - t);
        const auto db = (b.t_ns >= t) ? (b.t_ns - t) : (t - b.t_ns);

        // 同距離なら「未来側 b を優先」する．過去優先にしたければ条件を逆に．
        return (db < da) ? b.x : a.x;
    }
    
    EnergyCal::~EnergyCal() {} //　デストラクタ
}
