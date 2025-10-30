# include "utils/cpu_manager.hpp"


void set_cpu_governor(const std::string& governor) {
    // CPUのガバナーを設定する関数
    std::string command = "sudo cpufreq-set -g" + governor;
    int ret = system(command.c_str());
    if (ret != 0) {
        std::cerr << "[Error] setting CPU governor to " << governor << std::endl;
    } else {
        std::cout << "[Info] CPU governor set to " << governor << std::endl;
    }
}