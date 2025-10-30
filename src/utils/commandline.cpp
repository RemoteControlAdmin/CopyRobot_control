# include "utils/commandline.hpp"

std::pair<std::string, int> parse_command_line(int argc, char* argv[]){
    if (argc < 4){
        std::cerr << "[Error] Not enough arguments" << std::endl;
        std::cerr << "[Info] 1:local or tailscale \n"
                  << "       2:Target CopyRobot name (e.g., cra1, crb1, crc1, cra2, crb2, crc2, cra3, crb3) \n"
                  << "       3:Monitor port number (e.g., 52222, 53222)"
        << std::endl;
        exit(1);
    }
    auto select_mode = argv[1];
    auto target_copyrobot_name = argv[2];
    std::string head_ip;
    // Set target CopyRobot IP address
    if (std::string(select_mode) == std::string("local")){
        head_ip = "192.168.11.";
    }
    else if (std::string(select_mode) == std::string("tailscale")){
        head_ip = "100.77.38.";
    }
    else{
        std::cerr << "[Error] Invalid mode" << std::endl;
        exit(1);
    }
    static const std::unordered_map<std::string, std::string> ip_suffixes = {
        {"cra1", "11"}, {"crb1", "21"}, {"crc1", "31"},
        {"cra2", "12"}, {"crb2", "22"}, {"crc2", "32"},
        {"cra3", "13"}, {"crb3", "23"}
    };
    auto it = ip_suffixes.find(target_copyrobot_name);
    if (it == ip_suffixes.end()) {
        std::cerr << "[Error] Invalid CopyRobot name" << std::endl;
        std::exit(1);
    }
    std::string target_copyrobot_ip = head_ip + it->second;

    // Get monitor port
    if (std::stoi(argv[3]) >= 56000 and std::stoi(argv[3]) <= 50000){
        std::cerr << "[Error] Invalid monitor port" << std::endl;
        exit(1);
    }
    auto monitor_port = std::stoi(argv[3]);
    std::cout << "[Info] Target CopyRobot IP: " << target_copyrobot_ip << std::endl;
    std::cout << "[Info] Monitor port: " << monitor_port << std::endl;
    return {target_copyrobot_ip, monitor_port};
}
