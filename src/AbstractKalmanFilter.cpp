#include "AbstractKalmanFilter.h"
#include <iostream>

using Eigen::MatrixXd;
using namespace std;


AbstractKalmanFilter::~AbstractKalmanFilter() {}

AbstractKalmanFilter::AbstractKalmanFilter() {
    //the process covariance matrix Q
    Q_ = MatrixXd(4, 4);

    //the initial transition matrix F_
    F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    noise_ax = 9;
    noise_ay = 9;
}

void AbstractKalmanFilter::ProcessMeasurement(FilterState &state, const MeasurementPackage &measurement_pack) {
    if(!IsProcessible(measurement_pack))
        return;

    if(!state.is_initialized_){
        Init(state, measurement_pack);
        return;
    }

    float dt = (measurement_pack.timestamp_ - state.previous_timestamp_) / 1000000.0;
    if(dt <= 0)
        return;

    state.previous_timestamp_ = measurement_pack.timestamp_;

    UpdateProcessCovarianceMatrix(dt);
    Predict(state);
    Update(state, measurement_pack.raw_measurements_);
}

void AbstractKalmanFilter::Init(FilterState &state, const MeasurementPackage &measurement_pack) {
    state.previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    state.is_initialized_ = true;
}

void AbstractKalmanFilter::Predict(FilterState &state) {
    state.x_ = F_ * state.x_;
    MatrixXd Ft = F_.transpose();
    state.P_ = F_ * state.P_ * Ft + Q_;
}

void AbstractKalmanFilter::UpdateProcessCovarianceMatrix(float dt) {
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt / 2;
    double dt_4 = dt_3 * dt / 2;

    //Modify the F matrix so that the time is integrated
    F_(0, 2) = dt;
    F_(1, 3) = dt;

    //Set the process covariance matrix Q
    Q_ << dt_4 * noise_ax, 0, dt_3 * noise_ax, 0,
          0, dt_4 * noise_ay, 0, dt_3 * noise_ay,
          dt_3 * noise_ax, 0, dt_2 * noise_ax, 0,
          0, dt_3 * noise_ay, 0, dt_2 * noise_ay;
}
