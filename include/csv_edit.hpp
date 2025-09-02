#ifndef CSV_EDIT_HPP
#define CSV_EDIT_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>

namespace csv_lib{

class Csvedit{
    /*
    csvにデータを書き込むクラス
    */
    private:
        std::string filename;
        std::ofstream file;

    public:
        Csvedit(const std::string &filename);

        void csv_write_headers(const std::vector<std::string> &headers);
        void csv_write_data(const std::pair<std::vector<int64_t>,std::vector<double>>  &data);
        
        ~Csvedit();
};


}

#endif // UDP_CONNECT_HPP  // 3. ここでガードを閉じます