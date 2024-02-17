/*
 * KalmanFilter.cpp
 *
 *  Created on: Aug 22, 2021
 *      Author: raing
 */

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
	// TODO Auto-generated constructor stub
	
}

KalmanFilter::KalmanFilter(double process_noise, double sensor_noise, double estimated_error, double intial_value) {
	/* The variables are x for the filtered value, q for the process noise,
	   r for the sensor noise, p for the estimated error and k for the Kalman Gain.
	   The state of the filter is defined by the values of these variables.

	   The initial values for p is not very important since it is adjusted
	   during the process. It must be just high enough to narrow down.
	   The initial value for the readout is also not very important, since
	   it is updated during the process.
	   But tweaking the values for the process noise and sensor noise
	   is essential to get clear readouts.

	   For large noise reduction, you can try to start from: (see http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/ )
	   q = 0.125
	   r = 32
	   p = 1023 //"large enough to narrow down"
	   e.g.
	   myVar = Kalman(0.125,32,1023,0);
	*/
	this->q = process_noise;
	this->r = sensor_noise;
	this->p = estimated_error;
	this->x = intial_value; //x will hold the iterated filtered value
}

double KalmanFilter::getFilteredValue(double measurement)
{
	/* Updates and gets the current measurement value */
	//prediction update
	//omit x = x
	this->p = this->p + this->q;

	//measurement update
	this->k = this->p / (this->p + this->r);
	this->x = this->x + this->k * (measurement - this->x);
	this->p = (1 - this->k) * this->p;

	return this->x;
}

void KalmanFilter::setParameters(double process_noise, double sensor_noise, double estimated_error)
{
	this->q = process_noise;
	this->r = sensor_noise;
	this->p = estimated_error;
}

void KalmanFilter::setParameters(double process_noise, double sensor_noise)
{
	this->q = process_noise;
	this->r = sensor_noise;
}

double KalmanFilter::getProcessNoise()
{
	return this->q;
}

double KalmanFilter::getSensorNoise()
{
	return this->r;
}

double KalmanFilter::getEstimatedError()
{
	return this->p;
}

KalmanFilter::~KalmanFilter()
{
	// TODO Auto-generated destructor stub
}

