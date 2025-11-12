# include "utils/cpu_manager.hpp"


void set_cpu_governor(const std::string& governor) {
    const int cpu_count = 4;  // CPUコア数
    bool success = true;

    for (int i = 0; i < cpu_count; ++i) {
        std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_governor";
        std::ofstream ofs(path);
        if (!ofs) {
            std::cerr << "Error: cannot open " << path << " (permission denied or invalid path)" << std::endl;
            success = false;
            continue;
        }
        ofs << governor;
        if (!ofs) {
            std::cerr << "Error: failed to write governor to " << path << std::endl;
            success = false;
        }
    }

    if (success) {
        std::cout << "All CPU governors set to \"" << governor << "\" successfully." << std::endl;
    } else {
        std::cerr << "Some CPU governors could not be set." << std::endl;
    }
}