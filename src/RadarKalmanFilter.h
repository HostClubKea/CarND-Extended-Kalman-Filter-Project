#ifndef EXTENDEDKF_RADARKALMANFILTER_H
#define EXTENDEDKF_RADARKALMANFILTER_H

#include "AbstractKalmanFilter.h"
#include "tools.h"

class RadarKalmanFilter : public AbstractKalmanFilter {
public:
    double eps;

    RadarKalmanFilter();

    virtual ~RadarKalmanFilter();

    bool IsProcessible(const MeasurementPackage &measurement_pack) override ;

protected:
    void Init(FilterState &state, const MeasurementPackage &measurement_pack) override;

    void Update(FilterState &state, const VectorXd &z) override;

private:
    Tools tools;

    VectorXd getPredictedState(const FilterState &state) const;

    VectorXd &NormalizeAngle(VectorXd &y) const;
};


#endif //EXTENDEDKF_RADARKALMANFILTER_H
