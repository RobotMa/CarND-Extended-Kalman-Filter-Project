#include <iostream>
#include <math.h>
#include "tools.h"

using Eigen::Vector4d;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    Vector4d vecRMSE(0, 0, 0, 0);
    if (estimations.size() != ground_truth.size() || estimations.size() < 1 )
    {
        return vecRMSE;
    }

    std::cout << "Size of estimation element is " << estimations[0].size() << std::endl;
    std::cout << "Size of ground_truth element is " << ground_truth[0].size() << std::endl;

    for(const auto& e : estimations)
    {
        std::cout << "Element : " << e << std::endl;
    }


    for(const auto& g : ground_truth)
    {
        std::cout << "Element : " << g << std::endl;
    }


    int n1 = estimations.size();

    for(int i = 0; i < n1; ++i)
    {
        vecRMSE += estimations.at(i) - ground_truth.at(i);
        vecRMSE = vecRMSE.array()*vecRMSE.array();
    }

    vecRMSE /= n1;
    vecRMSE = vecRMSE.array().sqrt();

    std::cout << "Final residual is " << vecRMSE << std::endl;

    return vecRMSE;

}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3 ,4);

        //recover state parameters
        float px = x_state(0);
        float py = x_state(1);
        float vx = x_state(2);
        float vy = x_state(3);

        //pre-compute a set of terms to avoid repeated calculation
        float c1 = px*px+py*py;
        float c2 = sqrt(c1);
        float c3 = (c1*c2);

        //check division by zero
        if(fabs(c1) < 0.0001)
        {
            std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
        }

        //compute the Jacobian matrix
        Hj << (px/c2), (py/c2), 0, 0,
             -(py/c1), (px/c1), 0, 0,
               py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    return Hj;
}
