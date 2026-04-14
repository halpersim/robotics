#include "eigen-5.0.0/Eigen/Dense"
#include <vector>
#include <cmath>
#include <iostream>

using namespace Eigen;


int main()
{
    MatrixXd A(2,4);
    A << 1,2,3,4,
        -1,-2,-3,-4;

    MatrixXd b = A.replicate(1, 2);
    MatrixXd c = A(placeholders::all, VectorXi::LinSpaced(8,0,4));

    std::cout << b << std::endl;
    std::cout << "c:\n" << c << std::endl;

    //std::cout << VectorXi::LinSpaced(8,0,3) << std::endl;

    MatrixXd r(4*2, 2);

    for (int i = 0; i<4; i++) {
        for (int k=0; k<2; k++) {
            r(i*2+k, 0) = A(0, i);
            r(i*2+k, 1) = A(1, i);
        }
    }
    std::cout  << r << std::endl;

    return 0;
}