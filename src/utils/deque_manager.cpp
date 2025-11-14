
# include "utils/deque_manager.hpp"

namespace utils{
    DequeManager::DequeManager(std::string target_copyrobot_ip):
        udp_communicator(deque_master, deque_copy, queue_mutex_master, queue_mutex_copy, deque_udpforce, queue_mutex_udpforce), // UDP通信の初期化
        force_get(deque_force, queue_mutex_force, target_copyrobot_ip),
        master_lp_out_(),
      master_lp_init_(false),
      master_lp_alpha_(0.5)   // とりあえず 0.2〜0.5 あたりを試す
        {
            master_last_data = std::vector<double>(6, 0.0);
            copy_last_data = std::vector<double>(6, 0.0);
            partner_master_last_data = std::vector<double>(6, 0.0);
            remote_copy_last_data = std::vector<double>(6, 0.0);
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

    std::tuple<std::vector<double>, std::vector<double>, int64_t> DequeManager::get_master_data(){
    /*
    * masterデータ取得 get master data
    */

    {
        std::lock_guard<std::mutex> lock(queue_mutex_master);
        if (!deque_master.empty()){
            // 生データ取得
            std::pair<std::vector<double>, int64_t> getdata = deque_master.front();
            temp_master_data = getdata.first;
            master_data.assign(temp_master_data.begin(), temp_master_data.begin() + 6);
            remote_copy_data.assign(temp_master_data.begin() + 6, temp_master_data.end());
            master_send_time = getdata.second;
            deque_master.pop_front();

            // -------------------------
            // ① ウォームアップ 200 サンプル
            // -------------------------
            if (tempcount < 200){
                // 最大遅延を更新
                int64_t d = cal_delay_time(master_send_time);
                if (sum < d){
                    sum = d;
                }

                // last 系は「最新の生データ」を記録しておく
                master_last_data      = master_data;
                master_last_send_data = master_send_time;
                remote_copy_last_data = remote_copy_data;

                tempcount++;

                // ウォームアップ中はそのまま生データを返す（必要ならここをホールドに変えてもOK）
                return {master_data, remote_copy_data, master_send_time};
            }

            // tempcount == 200 のタイミングで一度だけ定常遅延を決める
            if (tempcount == 200){
                constexpr int64_t ten_ms = 0 * 1000 * 1000;  // 10ms in ns
                int64_t target_delay = sum + ten_ms;          // max + 10ms

                // 「現在の生データ時刻 − (max + 10ms)」までさかのぼった時刻からスタート
                last_target_time = master_send_time - target_delay;

                // 一度だけ通る
                tempcount++;
            }

            // ここまで来た時点で，
            // - master_last_* には前回の「生データ」
            // - master_* には今回の「生データ」
            // が入っているはず（ウォームアップ終了後）
        }
        else {
            not_get_count[0]++;

            // データが来ないとき，ウォームアップ中ならとりあえずホールド
            if (tempcount <= 200){
                master_data      = master_last_data;
                remote_copy_data = remote_copy_last_data;
                return {master_data, remote_copy_data, master_send_time};
            }

            // ウォームアップ後で新しい生データも無いなら，
            // 単純に前回の出力を 10ms 進めてそのまま返す（ホールド）
            target_time      = last_target_time + 10 * 1000 * 1000;
            last_target_time = target_time;
            return {master_data, remote_copy_data, master_send_time};//return {test_master, remote_copy_data, target_time};
        }
    } // unlock 用

    // -----------------------------
    // ② ウォームアップ後の補間処理
    // -----------------------------
    // master_send_time : 今回の生データ時刻
    // master_last_send_data : 前回の生データ時刻

    // 生データの時刻が進んでいないなら，補間のしようがないので「出力だけ 10ms 進めてホールド」
    if (master_send_time <= master_last_send_data){
        target_time      = last_target_time + 10 * 1000 * 1000;
        last_target_time = target_time;
        return {master_data, remote_copy_data, master_send_time};//return {test_master, remote_copy_data, target_time};
    }

    // ★ここが重要：次の出力時刻は必ず「前回の target_time + 10ms」
    target_time = last_target_time + 10 * 1000 * 1000;

    // 線形補間係数 α（外挿も許可）
    double alpha = static_cast<double>(target_time - master_last_send_data) /
                   static_cast<double>(master_send_time - master_last_send_data);

    for (std::size_t i = 0; i < master_data.size(); ++i) {
        test_master[i] =
            (1.0 - alpha) * master_last_data[i] +
            alpha         * master_data[i];
    }

    // ★ここから：1次IIRローパス（EMA）を test_master に適用
    if (!master_lp_init_) {
        // 初回はそのまま採用
        master_lp_out_  = test_master;
        master_lp_init_ = true;
    } else {
        // サイズが変わっていたらリセット（安全策）
        if (master_lp_out_.size() != test_master.size()) {
            master_lp_out_ = test_master;
        } else {
            const double a = master_lp_alpha_;  // 0< a <=1
            for (std::size_t i = 0; i < test_master.size(); ++i) {
                master_lp_out_[i] =
                    a * test_master[i] + (1.0 - a) * master_lp_out_[i];
            }
        }
    }
    // ★ここまで IIR

    // remote_copy はとりあえずゼロ次ホールド（必要なら同様に補間）
    remote_copy_last_data = remote_copy_data;

    // 状態更新
    std::cout << "period(ms): "
              << static_cast<double>(target_time - last_target_time) / 1'000'000.0
              << std::endl;

    last_target_time      = target_time;
    master_last_data      = master_data;
    master_last_send_data = master_send_time;

    // ★IIR後のデータを返す
    return {master_data, remote_copy_data, master_send_time};//eturn {master_lp_out_, remote_copy_data, target_time};
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
        std::cout << "[Info] Stopping DequeManager threads..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(4)); // 4秒待機 wait for 4 seconds
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
