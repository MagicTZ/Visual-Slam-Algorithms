#include <iostream>

#include "MyBA.h"
#include "BAL.h"

using namespace std;

const string bal_file = "./../problem-16-22106-pre.txt"; //平差文件

int main(int argc, char **argv) {
    // check if input rightly
    string bal_input;
    // bal_input = *argv[1];
    // 命令行参数(首选)或者函数内部给定文件路径
    if (argc != 2) {
        if (!bal_file.empty()) {
            bal_input = bal_file;
        } else {
            cout << "Input format wrong! The format: binaryfile  bal_data.txt" << endl;
            return 1;
        }
    }

    // Read BAL dat file
    BAL bal(bal_input); // 定义一个bal对象
    // Initial Status
    bal.WriteToPLY("initial.ply");
    // Normalize
    bal.Normalize();
    // Add noise by Perturb
    bal.Perturb();

    // BA algorithm
    bal.Optimization(50);
    //MyBA BA(bal);

    // Bundle Adjustment
    //BA.Optimization();

    return 0;
}