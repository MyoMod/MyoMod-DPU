/*
 * GestureEstimator.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: leon
 */

#include <GestureEstimator.h>

namespace freesthetics {

GestureEstimator::GestureEstimator():
		axes{0.5, 0.5, 0,1,0,1},
		activeAxes{false, false, false, false, false, false}
	// TODO Auto-generated constructor stub
{
}

GestureEstimator::~GestureEstimator() {
	// TODO Auto-generated destructor stub
}

Status GestureEstimator::update(std::span<const float32_t> data) {
	
	// check which axes are active
	for (uint8_t i = 0; i < 6; i++) {
		if(activeAxes[i]) {
			activeAxes[i] = (data[i] > hystFalling);
		}
		else {
			activeAxes[i] = (data[i] > hystRising);
		}
	}

	// calculate flex/extension
	float32_t flexSum = 0;
	for (uint8_t i = 3; i < 6; i++) {
		if (activeAxes[i]) {
			flexSum += data[i] * 0.333;
		}
	}
	for (uint8_t i = 1; i < 3; i++) {
		if (activeAxes[i]) {
			flexSum -= data[i] * 0.5;
		}
	}

	// make cubic scale with speedFactor
	flexSum = speedFactor * flexSum * flexSum * flexSum;

	// update axes
	for (uint8_t i = 0; i < 6; i++) {
		const HandGesture &gesture = gestures[activeGesture];
		if (gesture.activeAxes[i]) {
			axes[i] += flexSum;

			axes[i] = MIN(gesture.axesMax[i], MAX(gesture.axesRest[i], axes[i]));
		}
	}

	// gesture selection
	bool currentState = data[3] > chn4Min && data[5] < chn6Max;

	// debounce
	if (currentState == debounceState) {
		debounceCounter = 0;
	}
	else {
		debounceCounter++;
		if (currentState && debounceCounter > debounceHighThreshold) {
			activeGesture++;
			if (activeGesture >= gestures.size()) {
				activeGesture = 0;
			}
			debounceState = true;
		}
		else if (!currentState && debounceCounter > debounceLowThreshold) {
			debounceState = false;
		}
	}
	return Status::Ok;
}

Status GestureEstimator::getAxes(HandAxes& axes) {
	for (size_t i = 0; i < 6; i++)
	{
		axes[i] = this->axes[i] * 100;
	}
	return Status::Ok;	
}


} /* namespace freesthetics */
