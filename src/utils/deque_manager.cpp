
# include "utils/deque_manager.hpp"

namespace utils{
    DequeManager::DequeManager(std::string target_copyrobot_ip):
        udp_communicator(deque_master, deque_copy, queue_mutex_master, queue_mutex_copy, deque_udpforce, queue_mutex_udpforce), // UDP通信の初期化
        force_get(deque_force, queue_mutex_force, target_copyrobot_ip)
        {
            master_last_data = std::vector<double>(6, 0.0);
            copy_last_data = std::vector<double>(6, 0.0);
            partner_master_last_data = std::vector<double>(6, 0.0);
        } //コンストラクタ

    void DequeManager::udp_thread_manager(){
        /*
        * 受信スレッドの開始 start receive thread
        */

        udp_thread_master = std::thread(
            &net_lib::UdpCommunicator::recive_thread_from_master,
            &udp_communicator);

        udp_thread_copy = std::thread(
            &net_lib::UdpCommunicator::recive_thread_from_copy,
            &udp_communicator);

        recive_thread_get_forcevalue = std::thread(
            &net_lib::UdpCommunicator::recive_thread_get_forcevalue,
            &udp_communicator);  // ← & を忘れずに
    } 
    void DequeManager::force_thread_manager(spi_lib::SPIService& spi_service){
        /*
        * 力センサー受信スレッドの開始 start force sensor receive thread
        */
        force_thread = std::thread(
            &robot_lib::ForceGet::force_get_thread,
            &force_get,
            std::ref(spi_service));  // 参照引数は std::ref
    }

    void DequeManager::rest_not_get_coount(){
        /*
        * データが取得できなかった回数のリセット reset not get data count
        */
        not_get_count = {0,0,0};
    }

    std::pair<std::vector<double>, int64_t> DequeManager::get_master_data(){
        /*
        * masterデータ取得 get master data
        */
        {
            std::lock_guard<std::mutex> lock(queue_mutex_master);
            if (!deque_master.empty()){
                std::pair<std::vector<double>, int64_t> getdata = deque_master.front();
                master_data = getdata.first;
                master_send_time = getdata.second;
                master_last_data = master_data;
                deque_master.pop_front();
            }
            else{
                master_data = master_last_data;
                not_get_count[0]++;
            }
        } // unlock用

        return {master_data, master_send_time};
    }

    std::pair<std::vector<double>, std::vector<double>> DequeManager::get_copy_data(){
        /*
        * copyデータ取得 get copy data
        */
        {
            std::lock_guard<std::mutex> lock(queue_mutex_copy);
            if (!deque_copy.empty()){
                temp_copy_data = deque_copy.front().first;
                copy_data.assign(temp_copy_data.begin(), temp_copy_data.begin()+6);
                partner_master_data.assign(temp_copy_data.begin()+6, temp_copy_data.end());
                copy_last_data = copy_data;
                partner_master_last_data = partner_master_data;
                deque_copy.pop_front();
            }
            else{
                copy_data = copy_last_data;
                partner_master_data = partner_master_last_data;
                not_get_count[1]++;
            }
        } // unlock用

        return {copy_data, partner_master_data};
    }

    std::pair<std::vector<double>, int64_t> DequeManager::get_udpforce_data(){
        /*
        * UDP力センサデータ取得 get force sensor udp data
        */
        {
            std::lock_guard<std::mutex> lock(queue_mutex_udpforce);
            if (!deque_udpforce.empty()){
                force_udp_values = deque_udpforce.front().first;
                force_send_time = deque_udpforce.front().second;
                deque_udpforce.pop_front();
            }
            else{
                force_udp_values = {0.0, 0.0, 0.0, 0.0};
                not_get_count[2]++;
            }
        }

        return {force_udp_values, force_send_time};
    }
    std::vector<double> DequeManager::get_actforce_data(){
        /*
        * 力センサーデータ取得 get force sensor data
        */
        {
            std::lock_guard<std::mutex> lock(queue_mutex_force);
            if (!deque_force.empty()){
                force_values = deque_force.front();
                deque_force.pop_front();
            }
            else{
                force_values = {0.0, 0.0, 0.0, 0.0};
            }
        }

        return force_values;
    }
    
    DequeManager::~DequeManager() noexcept {
        auto safe_join = [](std::thread& th){
            if (th.joinable()) th.join();
        };
        safe_join(udp_thread_master);      // UDP receive thread from master
        safe_join(udp_thread_copy);        // UDP receive thread from copy
        safe_join(force_thread);           // force get thread
        safe_join(recive_thread_get_forcevalue);

        std::cout << "[Info] Not get data count: Master = " << not_get_count[0]
                << ", Copy = " << not_get_count[1]
              << ", Force = " << not_get_count[2] << std::endl;
    }
}
