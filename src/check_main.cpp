#include <iostream>
#include <vector>
#include <thread>  // ← 追加
#include <unistd.h>
#include <chrono>
#include "spi/spi_service.hpp"

int main() {
    spi_lib::SPIService spi_service;

    float V1 = 12.0f;
    float V2 = 12.0f;
    float V3 = 12.0f;
    //usleep(1000); // 少し待つ

    // --- ADC読み取りスレッド（最小：スレッド作成のみ） ---

    std::thread adc_thread([&] {
        using clock = std::chrono::steady_clock;
        const auto T = std::chrono::microseconds(1000);  // 1ms周期
    
        spi_service.init_adc();
    
        auto next = clock::now();
        auto last = next;
    
        while (true) {
            // --- ADC読み取り ---
            const auto values = spi_service.read_adc();
    
            // --- 実時間dt計測 ---
            auto now = clock::now();
            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last);
            last = now;
    
            // --- 表示 ---
            std::cout << "force values: "
                    << values[0] << ", " << values[1] << ", "
                    << values[2] << ", " << values[3] << " | "
                    << "dt = " << dt.count() << " us" << std::endl;
    
            // --- 次の周期まで待機 ---
            next += T;
            std::this_thread::sleep_until(next);
        }
    });


    
    adc_thread.detach();  // メイン無限ループなので detach

    
    // --- メインループ（モータのみ） ---
    using clock = std::chrono::steady_clock;
const auto T = std::chrono::microseconds(10'000);  // 10 ms周期（＝100 Hz）

auto next = clock::now();
auto last = next;

while (true) {
    V1 -= 0.01f;
    V2 -= 0.01f;
    V3 -= 0.01f;
    // --- モータ送信 ---
    bool result = spi_service.pico2_motor(V1, V2, V3);
    if (!result) {
        std::cout << "[ERR] Motor control failed." << std::endl;
    }

    // --- 実時間 dt 計測 ---
    auto now = clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last);
    last = now;

    // --- ログ出力 ---
    std::cout << "[MOTOR] dt = " << dt.count() << " us" << std::endl;

    // --- 次の周期まで待機 ---
    next += T;
    std::this_thread::sleep_until(next);
}


    
}
