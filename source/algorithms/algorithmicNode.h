/**
 * @file algorithmicNode.h
 * @author Leon Farchau (leon2225)
 * @brief Base node for all algorithms
 * @version 0.1
 * @date 15.07.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include "node.h"


class AlgorithmicNode : public BaseNode
{
public:
    virtual void process() = 0;
};