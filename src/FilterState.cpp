#include "FilterState.h"
using Eigen::VectorXd;
using Eigen::MatrixXd;

FilterState::FilterState() {
    x_ = VectorXd(4);

    //state covariance matrix P
    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

    is_initialized_ = false;
    previous_timestamp_ = 0;
}
