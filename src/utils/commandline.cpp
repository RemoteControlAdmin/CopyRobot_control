# include "utils/commandline.hpp"


std::string get_my_name(){
    /* ホスト名（raspi-以下）を抽出する関数*/
    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) != 0){
        std::cerr << "[Error] getting hostname" << std::endl;
        exit(1);
    }

    std::string hostname_str(hostname);
    return (hostname_str.substr(6)).substr(0,4);
}


std::tuple<std::string, std::string, int> parse_command_line(int argc, char* argv[]){
    if (argc < 5){
        std::cerr << "[Error] Not enough arguments" << std::endl;
        std::cerr << "[Info] 1:local or tailscale \n"
                  << "       2:Target CopyRobot name 1 (e.g., cra1, crb1, crc1, cra2, crb2, crc2, cra3, crb3) \n"
                  << "       3:Target CopyRobot name 2 (e.g., cra1, crb1, crc1, cra2, crb2, crc2, cra3, crb3) \n"
                  << "       4:Monitor port number (e.g., 50333, 51333, 52333)"
        << std::endl;
        exit(1);
    }
    auto select_mode = argv[1];
    auto target_copyrobot_name_1 = argv[2];
    auto target_copyrobot_name_2 = argv[3];
    std::string head_ip;
    // Set local or tailscale
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
    // Set Copy Robot IP 
    static const std::unordered_map<std::string, std::string> ip_suffixes = {
        {"cra1", "11"}, {"crb1", "21"}, {"crc1", "31"},
        {"cra2", "12"}, {"crb2", "22"}, {"crc2", "32"},
        {"cra3", "13"}, {"crb3", "23"}
    };
    auto it_1 = ip_suffixes.find(target_copyrobot_name_1);
    if (it_1 == ip_suffixes.end()) {
        std::cerr << "[Error] Invalid CopyRobot name" << std::endl;
        std::exit(1);
    }
    else if (target_copyrobot_name_1 == get_my_name()){
        std::cerr << "[Error] Target CopyRobot name cannot be the same as this robot" << std::endl;
        std::exit(1);
    }
    std::string target_copyrobot_ip_1 = head_ip + it_1->second;   
    
    auto it_2 = ip_suffixes.find(target_copyrobot_name_2);
    if (it_2 == ip_suffixes.end()) {
        std::cerr << "[Error] Invalid CopyRobot name" << std::endl;
        std::exit(1);
    }
    else if (target_copyrobot_name_2 == get_my_name()){
        std::cerr << "[Error] Target CopyRobot name cannot be the same as this robot" << std::endl;
        std::exit(1);
    }
    else if (target_copyrobot_name_2 == target_copyrobot_name_1){
        std::cerr << "[Error] Target CopyRobot name cannot be the same as CopyRobot name 1" << std::endl;
        std::exit(1);
    }
    std::string target_copyrobot_ip_2 = head_ip + it_2->second;   

    // Get monitor port
    if (std::stoi(argv[3]) >= 56000 and std::stoi(argv[3]) <= 50000){
        std::cerr << "[Error] Invalid monitor port" << std::endl;
        exit(1);
    }
    auto monitor_port = std::stoi(argv[3]);
    std::cout << "[Info] Target CopyRobot IP 1: " << target_copyrobot_ip_1 << std::endl;
    std::cout << "[Info] Target CopyRobot IP 2: " << target_copyrobot_ip_2 << std::endl;
        std::cout << "[Info] Monitor port: " << monitor_port << std::endl;
    return {target_copyrobot_ip_1, target_copyrobot_ip_2, monitor_port};
}
