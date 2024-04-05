#pragma once
#include <span>
#include <vector>
#include <cassert>
#include "arm_math.h"
#include "Status.h"

namespace freesthetics {


////////////////////////////////////////////////////////////////////////
// Typedefs
////////////////////////////////////////////////////////////////////////
typedef uint8_t HandAxes[6];


/*
 *
 */
class GestureEstimator {
public:
	GestureEstimator(){};
	virtual ~GestureEstimator() = default;

	virtual Status update(std::span<const float32_t> data) = 0;
	virtual Status getAxes(HandAxes& axes) const = 0;
};

} /* namespace freesthetics */

