#ifndef UDP_CONNECT_HPP
#define UDP_CONNECT_HPP

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>  // EXIT_FAILURE, exitのため
#include <cstdio>   // perrorのため
#include <iostream>
#include <vector>
#include <iterator>
#include <chrono>
#include <mutex>
#include <deque>
#include <atomic>

#include <common.hpp>

namespace udp_lib{

class UdpConnect{
    /*
    * ===============UDP送受信設定クラス==============
    * UDP communication settings class
    */
    int sock;
    struct sockaddr_in addr;
    size_t buffer_size;
    size_t total_buffer_size;
    char* buffer;

    public:

        UdpConnect(std::string address, int port, size_t element_count); // UDPコンストラクタ       
        
        void udp_send(const std::vector<double>& values , long roop_count); // UDP送信関数

        void udp_bind(); 

        std::pair<std::vector<double>, long> udp_recv();

        
        ~UdpConnect();
};

class UdpCommunicator{
    /*
    * ===============UDP通信処理クラス==============
    * UDP communication processing class
    */
   public:
        UdpCommunicator(std::deque<std::pair<std::vector<double>, long>>& deque_master, // address of deque_master
            std::deque<std::pair<std::vector<double>, long>>& deque_copy,
            std::mutex& queue_mutex_master,
            std::mutex& queue_mutex_copy);

        void recive_thread_from_master();
        
        void recive_thread_from_copy();

        void send_function();
        
        ~UdpCommunicator();

    private:
        std::deque<std::pair<std::vector<double>, long>>& deque_master_;
        std::deque<std::pair<std::vector<double>, long>>& deque_copy_;
        std::mutex& queue_mutex_master_;
        std::mutex& queue_mutex_copy_;
};

}

#endif // UDP_CONNECT_HPP  // 3. ここでガードを閉じます