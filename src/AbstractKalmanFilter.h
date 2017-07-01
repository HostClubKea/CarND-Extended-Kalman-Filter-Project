#ifndef EXTENDEDKF_ABSTRACTKALMANFILTER_H
#define EXTENDEDKF_ABSTRACTKALMANFILTER_H

#include <iostream>
#include "Eigen/Dense"
#include "FilterState.h"
#include "measurement_package.h"

using Eigen::VectorXd;
using namespace std;

class AbstractKalmanFilter {
public:

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    // acceleration noise components
    int noise_ax;

    int noise_ay;
    /**
     * Constructor
     */
    AbstractKalmanFilter();

    /**
     * Destructor
     */
    virtual ~AbstractKalmanFilter();

    void ProcessMeasurement(FilterState &state, const MeasurementPackage &measurement_pack);

    virtual bool IsProcessible(const MeasurementPackage &measurement_pack)  = 0;

protected:
    virtual void Init(FilterState &state, const MeasurementPackage &measurement_pack);

    void Predict(FilterState &state);

    virtual void Update(FilterState &state, const VectorXd &z) = 0;

    void UpdateProcessCovarianceMatrix(float dt);

};

#endif //EXTENDEDKF_ABSTRACTKALMANFILTER_H
