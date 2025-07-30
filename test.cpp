
// インクルード
#include <iostream>
#include <vector>

// a,b がクラス内変数にしたい
// cはクラスを呼び出す際にmain文から指定
Test::Test() {}


// 返り値として，計算結果を返してほしい
// 割り算は小数点以下も含めるように
// *voidと書いているのは一旦置いているだけ
// dは関数を呼び出す際にmain文から指定
void Test::sum() {

}  

void Test::difference() {

}

void Test::product() {

}
void Test::quotient() {

}
void Test::remainder() {

}
Test::~Test() {
    std::cout << "Test class destroyed" << std::endl;
}