//
// Created by dmitr on 30.06.2017.
//

#ifndef EXTENDEDKF_LIDARKALMANFILTER_H
#define EXTENDEDKF_LIDARKALMANFILTER_H


#include "AbstractKalmanFilter.h"

class LidarKalmanFilter : public AbstractKalmanFilter{
public:
    LidarKalmanFilter();

    virtual ~LidarKalmanFilter();

    bool IsProcessible(const MeasurementPackage &measurement_pack) override;

private:
    void Init(FilterState &state, const MeasurementPackage &measurement_pack) override;

protected:
    void Update(FilterState &state, const VectorXd &z) override;

};


#endif //EXTENDEDKF_LIDARKALMANFILTER_H
