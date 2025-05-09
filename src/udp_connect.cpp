#include "udp_connect.hpp"

namespace udp_lib {
/*
* ===============UDP送受信設定クラス==============
* UDP communication settings class
* https://planet-louse-95d.notion.site/1c047abc426580638ceff46276d2df59?pvs=4
*/
// コンストラクタ
UdpConnect::UdpConnect(std::string address, int port, size_t element_count) {
    /*
    UDP通信する相手のIPアドレスとポート番号が引数
    */
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50000;  // 50ミリ秒（必要に応じて調整）
    
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0) {
        perror("Error setting socket timeout");
        exit(EXIT_FAILURE);
    }

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(address.c_str());
    addr.sin_port = htons(port);
    
    //bufferの値を定義
    buffer_size = element_count * sizeof(double);
    total_buffer_size = buffer_size + sizeof(int);
    buffer = new char [total_buffer_size];
}

// UDP送信関数（double型データを送信）
void UdpConnect::udp_send(const std::vector<double>& values, int roop_count) {
    // valuesの値をbafferにコピー
    std::memcpy(buffer, values.data(), buffer_size);
    // nano_system_clockをbafferの末尾にコピー
    std::memcpy(buffer + values.size() * sizeof(double), &roop_count, sizeof(int)); // ＋で末尾に移動
    sendto(sock, buffer, total_buffer_size, 0, (struct sockaddr *)&addr, sizeof(addr));
}

// UDPバインド関数
void UdpConnect::udp_bind() {
    if (bind(sock, (const struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }
}

// UDP受信関数（double型データを受信）
std::pair<std::vector<double>, int> UdpConnect::udp_recv() {
    struct sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);
    //データ受信
    size_t received_bytes = recvfrom(sock, buffer, total_buffer_size, 0, (struct sockaddr*)&sender_addr, &addr_len);

    if (received_bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // タイムアウト：データなし
            return {};  // 空データを返す
        } else {
            perror("recvfrom failed");
            exit(EXIT_FAILURE);
        }
    }

    // 受信データのサイズが設定済みのバッファサイズと一致するか確認
    if (received_bytes != total_buffer_size) {
        std::cerr << "Error: Received data size mismatch!" << std::endl;
        return {};
    }
    // 受信データをstd::vector<double>に変換
    std::vector<double> received_values(buffer_size / sizeof(double));
    std::memcpy(received_values.data(), buffer, buffer_size);

    int roop_count;
    std::memcpy(&roop_count, buffer + buffer_size, sizeof(int));
    return {received_values, roop_count};
}

// デストラクタ
UdpConnect::~UdpConnect() {
    delete[] buffer;  // 動的に確保したメモリを解放
    close(sock);
}

/*
* ===============UDP通信処理クラス==============
* UDP communication processing class
*/
// コンストラクタの実装
UdpCommunicator::UdpCommunicator(std::deque<std::pair<std::vector<double>, int>>& deque_master, // address of deque_master
                                std::deque<std::pair<std::vector<double>, int>>& deque_copy,
                                std::mutex& queue_mutex_master,
                                std::mutex& queue_mutex_copy) :
                                deque_master_(deque_master), deque_copy_(deque_copy),
                                queue_mutex_master_(queue_mutex_master), queue_mutex_copy_(queue_mutex_copy) {}


void UdpCommunicator::recive_thread_from_master(){
    UdpConnect udpConnection_from_master("0.0.0.0", 40000, 6); // from Master Robot
    udpConnection_from_master.udp_bind();
    
    while(!stop_flag){
        std::pair<std::vector<double>, int> receiveddata_master = udpConnection_from_master.udp_recv(); // from master robot
        
        if (receiveddata_master.first.empty()) {
            continue;  // 空データならスキップ
        }
        
        {
            std::lock_guard<std::mutex> lock(queue_mutex_master_); // lock
            if (!deque_master_.empty()){
                deque_master_.pop_front();
            }
            deque_master_.push_back(receiveddata_master);
        }// unlock
    }
}

void UdpCommunicator::recive_thread_from_copy(){
    UdpConnect udpConnection_from_copy("0.0.0.0", 41000, 6); // from Copy Robot
    udpConnection_from_copy.udp_bind();

    while(!stop_flag){
        std::pair<std::vector<double>, int> receiveddata_copy = udpConnection_from_copy.udp_recv();     // from copy robot (own)

        if (receiveddata_copy.first.empty()) {
            continue;  // 空データならスキップ
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_copy_); // lock
            if (!deque_copy_.empty()){
                deque_copy_.pop_front();
            }
            deque_copy_.push_back(receiveddata_copy);}
        } // unlock
}

void UdpCommunicator::send_function(){
    UdpConnect udpConnection_raspberrypi("192.168.11.29", 4102, 6); // UDP初期化
}
// デストラクタの実装
UdpCommunicator::~UdpCommunicator() {
    // 必要なクリーンアップ処理をここに追加
}

} // namespace udp_lib
