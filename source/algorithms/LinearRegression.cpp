/*
 * LinearRegression.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: leon
 */

#include <LinearRegression.h>
#include <config.h>

namespace freesthetics {

LinearRegression::LinearRegression():
		axes{0.5, 0.5, 0,1,0,1}
	// TODO Auto-generated constructor stub
{
}

LinearRegression::~LinearRegression() {
	// TODO Auto-generated destructor stub
}

Status LinearRegression::update(std::span<const float32_t> data) {
	const uint32_t PREDICTION_N = CONFIG_COEF_PERSON1.size();
	assert((CONFIG_COEF_PERSON1[0].size() - 1) == data.size());

	std::array<float32_t, PREDICTION_N> yPrediction;


	// predict axes
	for (uint8_t i = 0; i < PREDICTION_N; i++) {
		yPrediction[i] = 0;
		// Check this, there is a warning here
		arm_dot_prod_f32(data.data(), CONFIG_COEF_PERSON1[i].data(), data.size(), &yPrediction[i]);
		yPrediction[i] += CONFIG_COEF_PERSON1[i][PREDICTION_N];
	}
	
	
	// check which predictions are active
	for (uint32_t i = 0; i < PREDICTION_N; i++) {
		if(activeAxes[i]) {
			activeAxes[i] = (yPrediction[i] > hystFalling);
		}
		else {
			activeAxes[i] = (yPrediction[i] > hystRising);
		}
	}

	// apply linear regression as velocity
	for (uint32_t i = 0; i < PREDICTION_N; i++) {
		if (activeAxes[i]) {
			// make cubic scale with speedFactor
			axes[i] += scale(yPrediction[i]);

			axes[i] = MIN(0.99, MAX(0.01, axes[i]));
		}
	}
	return Status::Ok;
}

Status LinearRegression::getAxes(HandAxes& axes) const{
	for (size_t i = 0; i < 6; i++)
	{
		axes[i] = this->axes[i] * 100;
	}
	return Status::Ok;	
}


} /* namespace freesthetics */
