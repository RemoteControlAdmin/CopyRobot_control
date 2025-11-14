# include "robot/force_get.hpp"
#include <fstream>  // ← 追加
#include <iomanip>  // ← setprecision で必要

namespace robot_lib { 
    ForceGet::ForceGet( 
        std::deque<std::vector<double>>& deque_force, std::mutex& queue_mutex_force, std::string target_copyrobot_ip):
        deque_force_(deque_force), queue_mutex_force_(queue_mutex_force),
        udpConnection_send_forcevalues(target_copyrobot_ip, 42000, 4)
        {
        notch_param_set();
        lowpass_param_set();
    } //コンストラクタ

    void ForceGet::lowpass_param_set(){
        double omega = 2.0 * M_PI * lowpass_fc / force_freq;
        double alpha = sin(omega) / (2.0 * Q);
        iir_a[0] = 1.0 + alpha;
        iir_a[1] = -2.0 * cos(omega);
        iir_a[2] = 1.0 - alpha;
        iir_b[0]  = (1.0 - cos(omega)) / 2.0;
        iir_b[1]  = 1.0 - cos(omega);
        iir_b[2]  = (1.0 - cos(omega)) / 2.0;
    }
    std::vector<double> ForceGet::filter_iirlowpass(std::vector<double> force_values){
        for(int i=0;i<4;i++){
            force_iir_x[i][0] = force_values[i];
            force_iir_y[i][0] = iir_b[0]/iir_a[0] * force_iir_x[i][0] + iir_b[1]/iir_a[0] * force_iir_x[i][1]
                              + iir_b[2]/iir_a[0] * force_iir_x[i][2]
                              - iir_a[1]/iir_a[0] * force_iir_y[i][1] - iir_a[2]/iir_a[0] * force_iir_y[i][2];
            force_iir_x[i][2] = force_iir_x[i][1];
            force_iir_x[i][1] = force_iir_x[i][0];
            force_iir_y[i][2] = force_iir_y[i][1];
            force_iir_y[i][1] = force_iir_y[i][0];
            force_values[i] = force_iir_y[i][0];
        }
        return force_values;
    }

    void ForceGet::notch_param_set(){
        double omega = 2.0 * M_PI * notch_fc / force_freq;
        double alpha = sin(omega) * sinh( log(2.0) / 2.0 * bw * omega / sin(omega) );

        notch_a[0] = 1.0 + alpha;
        notch_a[1] = -2.0 * cos(omega);
        notch_a[2] = 1.0 - alpha;
        notch_b[0]  = 1.0;
        notch_b[1]  = -2.0 * cos(omega);
        notch_b[2]  = 1.0;
    }

    std::vector<double> ForceGet::filter_iirnotch(std::vector<double> force_values){
        for(int i=0;i<4;i++){
            force_notch_x[i][0] = force_values[i];
            force_notch_y[i][0] = notch_b[0]/notch_a[0] * force_notch_x[i][0] + notch_b[1]/notch_a[0] * force_notch_x[i][1] 
                                + notch_b[2]/notch_a[0] * force_notch_x[i][2]
                                - notch_a[1]/notch_a[0] * force_notch_y[i][1] - notch_a[2]/notch_a[0]*force_notch_y[i][2];
            force_notch_x[i][2] = force_notch_x[i][1];
            force_notch_x[i][1] = force_notch_x[i][0];
            force_notch_y[i][2] = force_notch_y[i][1];
            force_notch_y[i][1] = force_notch_y[i][0];
            force_values[i] = force_notch_y[i][0];
        }
        return force_values;
    }
    
    void ForceGet::force_get_thread(spi_lib::SPIService& spi_service){ // 関数（任意の名前）

        spi_service.init_adc();
        using clock = std::chrono::steady_clock;
        const auto T = std::chrono::microseconds(force_freq);  // 1ms周期

        auto next = clock::now();
        auto last = next;

        //std::ofstream csv_file("force_data.csv");
        //csv_file << "time,force0,force1,force2,force3\n";

        while(!stop_flag){
            std::vector<double> force_values = spi_service.read_adc();
            force_values = filter_iirnotch(force_values);
            force_values = filter_iirlowpass(force_values);
            {
                std::lock_guard<std::mutex> lock(queue_mutex_force_);
                if (!deque_force_.empty()){
                    deque_force_.pop_front();
                }
                deque_force_.push_back(force_values);
            }
            std::chrono::high_resolution_clock::time_point current_clock = std::chrono::high_resolution_clock::now();
            udpConnection_send_forcevalues.udp_send(force_values, std::chrono::duration_cast<std::chrono::nanoseconds>(current_clock.time_since_epoch()).count());
            
            //std::chrono::high_resolution_clock::time_point current_clock = std::chrono::high_resolution_clock::now();
            //csv_file << std::chrono::duration_cast<std::chrono::nanoseconds>(current_clock.time_since_epoch()).count() << ","
            //         << force_values[0] << ","
            //         << force_values[1] << ","
            //         << force_values[2] << ","
            //         << force_values[3] << "\n";
            auto now = clock::now();
            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last);
            last = now;
            next += T;
            std::this_thread::sleep_until(next);
        }
        //csv_file.close();
    }
}