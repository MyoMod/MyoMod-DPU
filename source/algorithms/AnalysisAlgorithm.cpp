/*
 * AnalysisAlgorithm.cpp
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#include "AnalysisAlgorithm.h"
#include "ComInterface.h"

namespace freesthetics {

AnalysisAlgorithm::AnalysisAlgorithm(std::string_view name) :
		name(name),
		pdsIn(nullptr),
		pdsOut(nullptr)
 {
}

AnalysisAlgorithm::~AnalysisAlgorithm() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief Sets the ComInterface that is used to communicate with the devices.
 * 
 * @param comInterface 	Pointer to the ComInterface that is used to communicate with the devices.
 */
void AnalysisAlgorithm::setComInterface(ComInterface* comInterface)
{
	this->comInterface = comInterface;
	comInterface->getAllPds(pdsIn, pdsOut);
}

/**
 * @brief Empty function that is called when the algorithm is run.
 * 			Shall be overridden by the inheriting class to implement the functionality.
 * 
 * @return Status 		Ok if the algorithm ran successfully, Error otherwise. 
 * 							(maybe overwrite with own status of the inheriting class)
 */
Status AnalysisAlgorithm::run()
{
	return Status::Ok;
}

/**
 * @brief Empty function that is called when the algorithm is started.
 * 			Should be overridden by the inheriting class if needed for setup.
 * 			It shall be save to call this function multiple times.
 * 
 * @return Status  		Ok if the algorithm was started successfully, Error otherwise. 
 * 							(maybe overwrite with own status of the inheriting class)
 */
Status AnalysisAlgorithm::start()
{
	return Status::Ok;
}

/**
 * @brief Empty function that is called when the algorithm is stopped.
 * 			Should be overridden by the inheriting class if needed for cleanup.
 * 			It shall be save to call this function multiple times.
 * 
 * @return Status 		Ok if the algorithm was stopped successfully, Error otherwise. 
 * 							(maybe overwrite with own status of the inheriting class) 
 */
Status AnalysisAlgorithm::stop()
{
	return Status::Ok;
}

} /* namespace freesthetics */
