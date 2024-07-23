#pragma once
/*
 * DirectControl.h
 *
 *  Created on: Apr 5, 2024
 *      Author: leon
 */


#include <GestureEstimator.h>



////////////////////////////////////////////////////////////////////////
// Typedefs
////////////////////////////////////////////////////////////////////////

typedef struct {
	float32_t axesRest[6];
	float32_t axesMax[6];
	bool activeAxes[6];
} HandGesture;

////////////////////////////////////////////////////////////////////////
// Gestures
////////////////////////////////////////////////////////////////////////

const std::array<HandGesture, 5> gestures = {
	{
		{ // Wrist Flexion
			{0.01, 0.01, 0.01, 0.01, 0.01, 0.01},
			{0.99, 0.01, 0.01, 0.01, 0.01, 0.01},
			{true, false, false, false, false, false}
		},
		{ // Wrist Pronation
			{0.01, 0.01, 0.01, 0.01, 0.01, 0.01},
			{0.01, 0.99, 0.01, 0.01, 0.01, 0.01},
			{false, true, false, false, false, false}
		},
		{ // Lateral Pinch
			{0.01, 0.01, 0.01, 0.01, 0.99, 0.99},
			{0.01, 0.01, 0.01, 0.99, 0.99, 0.99},
			{false, false, true, true, true, true}
		},
		{ // Precision Pinch
			{0.01, 0.01, 0.99, 0.01, 0.01, 0.99},
			{0.01, 0.01, 0.99, 0.7, 0.7, 0.99},
			{false, false, true, true, true, true}
		},
		{ // Power Grip
			{0.01, 0.01, 0.99, 0.01, 0.01, 0.01},
			{0.01, 0.01, 0.99, 0.99, 0.99, 0.99},
			{false, false, true, true, true, true}
		}
	}
};

/*
 *
 */
class DirectControl : public GestureEstimator{
public:
	DirectControl();
	virtual ~DirectControl();

	Status update(std::span<const float32_t> data);
	Status getAxes(HandAxes& axes) const;
	uint32_t getActiveGesture() const { return activeGesture; }
private:
	std::array<float32_t, 6> axes;
	std::array<bool, 6> activeAxes;
	float32_t hystRising = 0.25;
	float32_t hystFalling = 0.15;
	float32_t speedFactor = 0.1;

	// Gesture selection

	uint32_t activeGesture = 2;
	float32_t chn4Min = 0.8;
	float32_t chn6Max = 0.40;
	
	// Debounce
	uint32_t debounceCounter = 0;
	uint32_t debounceHighThreshold = 5;
	uint32_t debounceLowThreshold = 20;
	bool debounceState = false;
};


