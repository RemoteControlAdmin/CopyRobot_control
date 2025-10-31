#pragma
#include <cmath>
#include <vector>
#define _USE_MATH_DEFINES  // M_PIを使用するために必要

# include <deque>
# include "common.hpp"
# include <optional>
# include <cstdint>
# include <chrono>
// 必要なライブラリのインクルード

namespace stability_lib { 

    class EnergyCal {

    public: // 以下public関数と変数の宣言

    EnergyCal(); // コンストラクタ
    ~EnergyCal(); // デストラクタ

    double loc_energy_cal(std::vector<double> copy_data, 
        std::vector<double> partner_master_data, std::vector<double> force_actual_data); // 関数（任意の名前）
    
    double sum_energy_cal(RobotData robotdata, int64_t send_time); // 関数（任意の名前）
    private: // 以下private関数と変数の宣言
    
    struct Rec {
        int64_t t_ns;
        double x;
    };


    std::deque<Rec> q_;
    static constexpr std::size_t K = 100;

    double sum_energy;


    void push(int64_t t_ns, double x);
    double nearest(int64_t t);

    }; 
} // namespace 
