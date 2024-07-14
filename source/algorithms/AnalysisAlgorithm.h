#pragma once
/*
 * AnalysisAlgorithm.h
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#include <string>
#include <string_view>

//#include "ComInterface.h"
#include "Status.h"



class ComInterface;
/*
 *
 */
class AnalysisAlgorithm {
public:
	AnalysisAlgorithm(std::string_view name);
	virtual ~AnalysisAlgorithm();

	void setComInterface(ComInterface* comInterface);

	virtual Status run() = 0;
	
	virtual Status start();
	virtual Status stop();

	std::string_view getName();
protected:
	std::string name;
	ComInterface* comInterface;
};



