#ifndef KALMANFILTERSCALE_H
#define KALMANFILTERSCALE_H

#include <Eigen/Dense>

#include <kalman/ekfilter.hpp>

class KalmanFilterScale : public Kalman::EKFilter<float,1> {
public:
	KalmanFilterScale();
	~KalmanFilterScale();

protected:
	void makeA();
    void makeH();
    void makeV();
    void makeR();
    void makeW();
    void makeQ();
    void makeProcess();
    void makeMeasure();

};










#endif
