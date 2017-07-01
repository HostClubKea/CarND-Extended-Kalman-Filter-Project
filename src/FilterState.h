#ifndef EXTENDEDKF_FILTERSTATE_H
#define EXTENDEDKF_FILTERSTATE_H
#include "Eigen/Dense"

class FilterState {
public:
    FilterState();

public:
    // check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    long previous_timestamp_;

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;
};


#endif //EXTENDEDKF_FILTERSTATE_H
