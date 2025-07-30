
# include "test.h"

int main(){
    // initialize variables
    int a = 1; // assuming a is defined in the context
    int b = 2; // assuming b is defined in the context 
    int c = 3; // assuming c is defined in the context
    int d = 4; // assuming d is defined in the context
    //　dは足し算は４，引き算は５，掛け算は６，割り算は７，あまりは８とする
    // perform basic arithmetic operations
    int sum = a + b + c + d; // assuming c and d are defined in the context
    int difference = a - b - c - d;
    int product = a * b * c * d; // assuming c and d are defined in the context
    int quotient = (a + b) / (c + d) ;
    int remainder = (a + b) / (c + d);

    // destructors and destructors
    a = 0;
    b = 0;
    c = 0;
    d = 0;
}