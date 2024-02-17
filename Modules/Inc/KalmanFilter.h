/*
 * KalmanFilter.h
 *
 *  Created on: Aug 22, 2021
 *      Author: raing
 */

#ifndef SRC_KALMANFILTER_H_
#define SRC_KALMANFILTER_H_
#include <stdint.h>

class KalmanFilter
{
public:
	KalmanFilter();
	KalmanFilter(double process_noise, double sensor_noise, double estimated_error, double intial_value);
	double getFilteredValue(double measurement);
	void setParameters(double process_noise, double sensor_noise, double estimated_error);
	void setParameters(double process_noise, double sensor_noise);
	double getProcessNoise();
	double getSensorNoise();
	double getEstimatedError();
	virtual ~KalmanFilter();

private:
	/* Kalman filter variables */
	double q; //process noise covariance
	double r; //measurement noise covariance
	double x; //value
	double p; //estimation error covariance
	double k; //kalman gain
};

#endif /* SRC_KALMANFILTER_H_ */
