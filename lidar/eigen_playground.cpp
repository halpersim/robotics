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

    MatrixXd b = A.replicate(2, 1);
    MatrixXd c = A(placeholders::all, VectorXi::LinSpaced(8,0,4));

    std::cout << b << std::endl;
    std::cout << "c:\n" << c << std::endl;
    

    std::cout << VectorXi::LinSpaced(8,0,3);
    return 0;
}