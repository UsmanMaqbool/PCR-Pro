#include "KalmanFilterScale.h"

KalmanFilterScale::KalmanFilterScale() {
	setDim(1, 0, 1, 3, 3); // dim states n, dim inputs nu, dim process noise nw, dim measurements m, dim measurement noise nv
}

KalmanFilterScale::~KalmanFilterScale() {}

void KalmanFilterScale::makeProcess() {
	// Process model
	Vector x_(x.size());
	x_(1) = x(1);
	x.swap(x_);
}

void KalmanFilterScale::makeMeasure() {
	// Measurement model
	z(1) = x(1);
	z(2) = x(1);
	z(3) = x(1);
}

void KalmanFilterScale::makeA() {
	// Jacobian of process model
	A(1,1) = 1.0;
}

void KalmanFilterScale::makeH() {
	// Jacobian of measurement model
	H(1,1) = 1.0;
	H(2,1) = 1.0;
	H(3,1) = 1.0;
}

void KalmanFilterScale::makeW() {
	// Jacobian of process noise
	W(1,1) = 1.0;
}

void KalmanFilterScale::makeV() {
	// Jacobian of Measurement noise
	V(1,1) = 1.0;
	V(2,2) = 1.0;
	V(3,3) = 1.0;
	V(2,1) = V(3,1) =
			V(1,2) = V(3,2) =
			V(1,3) = V(2,3) = 0.0;
}

void KalmanFilterScale::makeQ() {
	// Covaricane of process noise
	Q(1,1) = 1.0;
}

void KalmanFilterScale::makeR() {
	// Covaricane of measurement noise
	R(1,1) = 1.0;
	R(2,2) = 1.0;
	R(3,3) = 1.0;

	R(2,1) = R(3,1) =
			R(1,2) = R(3,2) =
			R(1,3) = R(2,3) = 0.0;
}
