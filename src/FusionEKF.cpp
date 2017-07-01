#include "FusionEKF.h"
#include "RadarKalmanFilter.h"
#include "LidarKalmanFilter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    AbstractKalmanFilter* radarKalmanFilter_ = new RadarKalmanFilter(); //RadarKalmanFilter radarKalmanFilter_;
    filters_.push_back(radarKalmanFilter_); //&radarKalmanFilter_);
    AbstractKalmanFilter* lidarKalmanFilter_ = new LidarKalmanFilter(); //LidarKalmanFilter lidarKalmanFilter_;
    filters_.push_back(lidarKalmanFilter_); //&lidarKalmanFilter_);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

    for(AbstractKalmanFilter* filter: filters_){
        filter->ProcessMeasurement(state_, measurement_pack);
    }

    // print the output
    cout << "x_ = " << state_.x_ << endl;
    cout << "P_ = " << state_.P_ << endl;

}