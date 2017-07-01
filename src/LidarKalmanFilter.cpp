#include "LidarKalmanFilter.h"
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

LidarKalmanFilter::~LidarKalmanFilter() {}

LidarKalmanFilter::LidarKalmanFilter() {
    // initializing matrices
    R_ = MatrixXd(2, 2);
    H_ = MatrixXd(2, 4);

    H_ << 1, 0, 0, 0,
          0, 1, 0, 0;

    //measurement covariance matrix - laser
    R_ << 0.0225, 0,
          0, 0.0225;

}

bool LidarKalmanFilter::IsProcessible(const MeasurementPackage &measurement_pack) {
    return measurement_pack.sensor_type_ == MeasurementPackage::LASER;
}

void LidarKalmanFilter::Init(FilterState &state, const MeasurementPackage &measurement_pack) {
    //Init filter state
    state.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    AbstractKalmanFilter::Init(state, measurement_pack);
}

void LidarKalmanFilter::Update(FilterState &state, const VectorXd &z) {
    VectorXd z_pred = H_ * state.x_;
    VectorXd y = z - z_pred;

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * state.P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = state.P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    state.x_ = state.x_ + (K * y);
    long x_size = state.x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    state.P_ = (I - K * H_) * state.P_;
}
