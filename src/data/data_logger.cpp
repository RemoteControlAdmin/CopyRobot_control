#include "data/data_logger.hpp"


namespace data_lib{
    
DataLogger::DataLogger(int monitor_port): csvWriter("output_file/" + get_my_name() + ".csv"),
                        csv_vector(38, 0.0),
                        udpConnection_monitor("100.124.38.52", monitor_port, 7)
                        {
    initialize_csv();
}

std::string  DataLogger::get_my_name(){
    /* ホスト名（raspi-以下）を抽出する関数*/
    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) != 0){
        std::cerr << "[Error] getting hostname" << std::endl;
        exit(1);
    }

    std::string hostname_str(hostname);
    return (hostname_str.substr(6)).substr(0,3);
}

void DataLogger::initialize_csv(){
    csvWriter.csv_write_headers({"TT","RT",
                            "TD","FTD",
                            "Perrx","Perry","Aerr", 
                            "Vcx","Vcy","Wc",
                            "Ww1","Ww2","Ww3",
                            "Vm1","Vm2","Vm3",
                            "FEactM","FEactA",
                            "FEvirM","FEvirA","VirDir",
                            "FIdeaM","FIdeaA","IdeDir", 
                            "FUDPM",
                            "SumEnergy","RemoteEnergy","LocalEnergy", //26
                            "CRx","CRy","CRt","MRx","MRy","MRt","RCRx","RCRy","RCRt","PMRx","PMRy","PMRt", // debug
                    });
}

void DataLogger::save_csv(RobotData robotdata, Mat3x1 mat3x1, int64_t send_clock, int64_t receive_clock, double delay_time, double force_delay_time, std::vector<double> energy){
    // デバック用 debug
    csv_vector.clear();
    csv_vector.push_back(delay_time);
    csv_vector.push_back(force_delay_time);
    //csv_vector.insert(csv_vector.end(), robotdata.err_data.begin(), robotdata.err_data.end()); // 3
    for (const auto& val : robotdata.err_data) {
        csv_vector.push_back(val * 1000);
    }
    //csv_vector.insert(csv_vector.end(), mat3x1.velocity_data.data(), mat3x1.velocity_data.data()+3); // 3
    for (int i = 0; i < mat3x1.velocity_data.size(); ++i) {
        double val = mat3x1.velocity_data[i];
        csv_vector.push_back(val * 1000);
    }
    csv_vector.insert(csv_vector.end(), mat3x1.invwheelvelocity.data(), mat3x1.invwheelvelocity.data()+3); // 3
    csv_vector.insert(csv_vector.end(), mat3x1.mortor_voltage.data(), mat3x1.mortor_voltage.data()+3); // 3
    csv_vector.insert(csv_vector.end(), robotdata.force_actual_data.begin(), robotdata.force_actual_data.begin()+2); // 2
    csv_vector.insert(csv_vector.end(), robotdata.force_virtual_data.begin(), robotdata.force_virtual_data.begin()+3); // 6
    csv_vector.insert(csv_vector.end(), robotdata.force_ideal_data.begin(), robotdata.force_ideal_data.begin()+3); // 6
    csv_vector.push_back(robotdata.force_udp_data[0]); // 1
    csv_vector.insert(csv_vector.end(), energy.begin(), energy.end()); // 1
    /* debug data
    */
    csv_vector.insert(csv_vector.end(), robotdata.copy_data.begin(), robotdata.copy_data.end()); // 3
    csv_vector.insert(csv_vector.end(), robotdata.master_data.begin(), robotdata.master_data.end()); // 3
    csv_vector.insert(csv_vector.end(), robotdata.remote_copy_data.begin(), robotdata.remote_copy_data.end()); // 3
    csv_vector.insert(csv_vector.end(), robotdata.partner_master_data.begin(), robotdata.partner_master_data.end()); // 3
    /*
    */
    csv_data = {{send_clock, receive_clock},csv_vector};
    csvWriter.csv_write_data(csv_data);
}


// データを表示する関数
void DataLogger::show_data(RobotData robotdata, Eigen::Vector3d velocity_data, int dt, double delay_time, double force_delay_time){
    std::cout << "\033[2J\033[1;1H"; // Clear the console
    std::cout << "================== Data  ==================" << std::endl;
    std::cout << std::left << std::setw(20) << ("Mpx = " + std::to_string(robotdata.master_data[0])) 
              << std::left << std::setw(20) << ("Mpy = " + std::to_string(robotdata.master_data[1]))
              << std::left << std::setw(20) << ("Mpt = " + std::to_string(robotdata.master_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("Cpx = " + std::to_string(robotdata.copy_data[0])) 
              << std::left << std::setw(20) << ("Cpy = " + std::to_string(robotdata.copy_data[1]))
              << std::left << std::setw(20) << ("Cpt = " + std::to_string(robotdata.copy_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("PMx = " + std::to_string(robotdata.partner_master_data[0])) 
              << std::left << std::setw(20) << ("PMy = " + std::to_string(robotdata.partner_master_data[1]))
              << std::left << std::setw(20) << ("PMt = " + std::to_string(robotdata.partner_master_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("Errx = " + std::to_string(robotdata.err_data[0])) 
              << std::left << std::setw(20) << ("Erry = " + std::to_string(robotdata.err_data[1]))
              << std::left << std::setw(20) << ("Errt = " + std::to_string(robotdata.err_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("Vx = " + std::to_string(velocity_data[0])) 
              << std::left << std::setw(20) << ("Vy = " + std::to_string(velocity_data[1]))
              << std::left << std::setw(20) << ("Vt = " + std::to_string(velocity_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("FEactM = " + std::to_string(robotdata.force_actual_data[0])) 
              << std::left << std::setw(20) << ("FEactA = " + std::to_string(robotdata.force_actual_data[1]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("FVirM = " + std::to_string(robotdata.force_virtual_data[0])) 
              << std::left << std::setw(20) << ("FVirA = " + std::to_string(robotdata.force_virtual_data[1]))
              << std::left << std::setw(20) << ("VirDir = " + std::to_string(robotdata.force_virtual_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("FIdeM = " + std::to_string(robotdata.force_ideal_data[0]))
              << std::left << std::setw(20) << ("FIdeA = " + std::to_string(robotdata.force_ideal_data[1]))
              << std::left << std::setw(20) << ("IdeDir = " + std::to_string(robotdata.force_ideal_data[2]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("FUDPM = " + std::to_string(robotdata.force_udp_data[0]))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("dt = " + std::to_string(dt))
              << std::endl;
    std::cout << std::left << std::setw(20) << ("Delay = " + std::to_string(delay_time)) 
              << std::endl;
    std::cout << std::left << std::setw(20) << ("Force Delay = " + std::to_string(force_delay_time))
              << std::endl;
    std::cout << "==============================================" << std::endl;
}

void DataLogger::send_monitor(RobotData robotdata, double delay_time, int64_t receive_clock){
    // UDPでモニタにデータを送信する関数
    udp_vector.clear();
    udp_vector.insert(udp_vector.end(), robotdata.force_actual_data.begin(), robotdata.force_actual_data.begin()+2); //
    udp_vector.insert(udp_vector.end(), robotdata.force_virtual_data.begin(), robotdata.force_virtual_data.begin()+2); 
    udp_vector.insert(udp_vector.end(), robotdata.force_ideal_data.begin(), robotdata.force_ideal_data.begin()+2); // 6
    udp_vector.push_back(delay_time);
    udpConnection_monitor.udp_send(udp_vector, receive_clock);
}

void DataLogger::move_data(){
    std::string local_file = "output_file/" + get_my_name() + ".csv";
    std::string remote_path = R"(/mnt/shared_csv/)" + get_my_name() + ".csv"; // 共有フォルダのパスを指定
    std::cout << "[Info] Moving file " << remote_path << std::endl;
    try {
        std::filesystem::copy_file(local_file, remote_path, std::filesystem::copy_options::overwrite_existing);
        // 移動にしたい場合は元ファイルを削除
        //std::filesystem::remove(local_file);
        std::cout << "[Info] File move completed" << std::endl;
    } catch (std::filesystem::filesystem_error& e) {
        std::cerr << "[Error] " << e.what() << std::endl;
    }
}

DataLogger::~DataLogger(){
    move_data();

}
}