#include <iostream>
#include <math.h>
#include "tools.h"

using Eigen::VectorXd;
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
    VectorXd vecRMSE;
    vecRMSE << 0, 0, 0, 0;
    if (estimations.size() != ground_truth.size() || estimations.size() < 1 )
    {
        return vecRMSE;
    }

    int n = estimations.size();

    for(int i = 0; i < estimations[0].size(); ++i)
    {
        for(int j = 0; j < n; ++j)
        {
            vecRMSE[i] += (estimations[i][j] - ground_truth[i][j])*(estimations[i][j] - ground_truth[i][j]);
        }
        vecRMSE[i] = 1.0/double(n)*sqrt(vecRMSE[i]);
    }

    std::cout << vecRMSE << std::endl;

    return vecRMSE;

}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd jacobian;

    return jacobian;
}
