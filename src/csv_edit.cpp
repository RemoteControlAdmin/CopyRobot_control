#include "csv_edit.hpp"
#include <iostream>
#include <fstream>


namespace csv_lib{
// コンストラクタ
Csvedit::Csvedit(const std::string &filename) : filename(filename) {
    // 既にファイルが存在する場合は削除
    if (std::ifstream(filename)) {
        if (std::remove(filename.c_str()) != 0) {
            std::cerr << "既存ファイルの削除に失敗しました: " << filename << std::endl;
        } else {
            std::cout << "既存ファイルを削除しました: " << filename << std::endl;
        }
    }

    // 新しくファイルを開く（上書きモード）
    file.open(filename, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "ファイルを開けませんでした: " << filename << std::endl;
    }
}

// ヘッダを設定する関数
void Csvedit::csv_write_headers(const std::vector<std::string> &headers) {

    if (!file.is_open()) return;

    // ヘッダを書き込む
    for (size_t i = 0; i < headers.size(); ++i) {
        file << headers[i];
        if (i < headers.size() - 1) {
            file << ","; // カンマで区切る
        }
    }
    file << "\n"; // ヘッダ行末に改行を追加
    file.flush();  // バッファの内容をファイルに書き込む
}

// データをCSVファイルに書き込むメソッド
void Csvedit::csv_write_data(const std::pair<std::vector<int64_t>,std::vector<double>>   &data) {

    if (!file.is_open()) {
        std::cerr << "ファイルを開けませんでした: " << filename << std::endl;
        return;
    }

    file << data.first[0];
    file << ",";
    file << data.first[1];
    file << ",";
    // vector<double>のデータを書き込み
    for (size_t i = 0; i < data.second.size(); ++i) {
        file << data.second[i];
        if (i < data.second.size()) {
            file << ","; // カンマで区切る
        }
    }
    file << "\n"; // 行末に改行を追加
    file.flush();  // バッファの内容をファイルに書き込む

}

Csvedit::~Csvedit(){
    if (file.is_open()){
        file.close();
    }
}

}