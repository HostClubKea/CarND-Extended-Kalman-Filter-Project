#include "RadarKalmanFilter.h"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;

RadarKalmanFilter::~RadarKalmanFilter() {}

RadarKalmanFilter::RadarKalmanFilter() {
    // initializing matrices
    R_ = MatrixXd(3, 3);
    H_ = MatrixXd(3, 4);

    //measurement covariance matrix - radar
    R_ << 0.09, 0, 0,
          0, 0.0009, 0,
          0, 0, 0.09;

    eps = 0.000001;
}

void RadarKalmanFilter::Update(FilterState &state, const VectorXd &z) {
    // Laser updates
    H_ = tools.CalculateJacobian(state.x_);

    VectorXd h = getPredictedState(state);
    VectorXd y = z - h;

    y = NormalizeAngle(y);

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

VectorXd &RadarKalmanFilter::NormalizeAngle(VectorXd &y) const {
    const double Max = M_PI;
    const double Min = -M_PI;

    y[1] = y[1] < Min
           ? Max + fmod(y[1] - Min, Max - Min)
           : fmod(y[1] - Min, Max - Min) + Min;
    return y;
}

VectorXd RadarKalmanFilter::getPredictedState(const FilterState &state) const {
    double px = state.x_(0);
    double py = state.x_(1);
    double vx = state.x_(2);
    double vy = state.x_(3);

    // Make sure we don't divide by 0.
    if (fabs(px) < eps && fabs(py) < eps) {
        px = eps;
        py = eps;
    } else if (fabs(px) < eps) {
        px = eps;
    }

    float ro = sqrtf(powf(px, 2) + powf(py, 2));
    float theta = atan2f(py, px);
    float ro_dot = (px * vx + py * vy) / ro;

    VectorXd h = VectorXd(3); // h(x_)
    h << ro, theta, ro_dot;
    return h;
}

bool RadarKalmanFilter::IsProcessible(const MeasurementPackage &measurement_pack) {
    return measurement_pack.sensor_type_ == MeasurementPackage::RADAR;
}

void RadarKalmanFilter::Init(FilterState &state, const MeasurementPackage &measurement_pack) {
    double ro = measurement_pack.raw_measurements_[0]; // range
    double theta = measurement_pack.raw_measurements_[1]; // bearing
    double ro_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
    //Convert radar from polar to cartesian coordinates
    double x = ro * cos(theta);
    double y = ro * sin(theta);
    double vx = ro_dot * cos(theta);
    double vy = ro_dot * sin(theta);
    //Init filter state
    state.x_ << x, y, vx, vy;

    AbstractKalmanFilter::Init(state, measurement_pack);
}
