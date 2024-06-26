#pragma once
/*
 * LinearRegression.h
 *
 *  Created on: Apr 5, 2024
 *      Author: leon
 */


#include <GestureEstimator.h>

namespace freesthetics {

////////////////////////////////////////////////////////////////////////
// Typedefs
////////////////////////////////////////////////////////////////////////


/*
 *
 */
class LinearRegression : public GestureEstimator{
public:
	LinearRegression();
	virtual ~LinearRegression();

	Status update(std::span<const float32_t> data);
	Status getAxes(HandAxes& axes) const;
private:
	std::array<float32_t, 6> axes;
	std::array<bool, 6> activeAxes;
	float32_t hystRising = 0.25;
	float32_t hystFalling = 0.15;
	float32_t speedFactor = 0.1;

	float32_t scale(float32_t x) const{
		return speedFactor * x;
	}
};

} /* namespace freesthetics */
