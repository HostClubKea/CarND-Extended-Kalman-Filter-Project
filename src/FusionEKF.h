#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <list>
#include "FilterState.h"
#include "AbstractKalmanFilter.h"

using namespace std;

class FusionEKF {
public:
    /**
    * Constructor.
    */
    FusionEKF();

    /**
    * Destructor.
    */
    virtual ~FusionEKF();

    /**
    * Run the whole flow of the Kalman Filter from here.
    */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
    * Kalman Filter update and prediction math lives in here.
    */
    FilterState state_;

    /**
    * Kalman Filter update and prediction math lives in here.
    */
    list<AbstractKalmanFilter*> filters_;

};

#endif /* FusionEKF_H_ */
