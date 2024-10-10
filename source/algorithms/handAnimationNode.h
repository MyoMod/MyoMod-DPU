/**
 * @file testAlgorithm.h
 * @author Leon Farchau (leon2225)
 * @brief A simple algorithm for testing the core system
 *          that mirrors the buttons of the bardisplay
 *          to the leds of the bardisplay
 *          
 * @version 0.1
 * @date 15.07.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <array>
#include "math.h"

#include "algorithmicNode.h"

class HandAnimationNode : public AlgorithmicNode
{
protected :
    std::array<std::shared_ptr<OutputPort<uint8_t>>, 8> m_handPoseOutputPorts;
    uint32_t m_stateProgress;
    uint32_t m_state;
    std::array<std::array<uint8_t, 8>, 4> m_states;
    std::array<uint32_t, 8> m_stateDurations;
public:
    HandAnimationNode(): 
        AlgorithmicNode(),
        m_stateProgress(0),
        m_state(0),
        m_states({{{ 0, 0, 0, 0, 0, 0, 0, 0 },
                  { 255, 255, 255, 255, 255, 255, 0 , 255},
                  { 255, 255, 255, 255, 255, 255, 30, 255},
                  { 255, 255, 255, 255, 255, 255, 0 , 255}}}),
        m_stateDurations({ 200, 20, 20, 20, 20, 200 })

    {
        for (size_t i = 0; i < m_handPoseOutputPorts.size(); i++)
        {
            m_handPoseOutputPorts[i] = std::make_shared<OutputPort<uint8_t>>();
            m_outputPorts.push_back(m_handPoseOutputPorts[i]);
        }
    }
    ~HandAnimationNode() = default;

    void process() override{
        // calculate the what the current target state should be (forward until the last state, then backwards)
        uint32_t currentStateIndex = (m_state > 3) ? (6 - m_state) : (m_state);
        uint32_t nextStateIndex = (m_state > 2) ? (5 - m_state) : (m_state + 1);

        // calculate the current state
        for (size_t i = 0; i < m_handPoseOutputPorts.size(); i++)
        {
            float startValue = m_states[currentStateIndex][i];
            float endValue = m_states[nextStateIndex][i];
            float progress = (float)m_stateProgress / (float)m_stateDurations[m_state];
            float value = startValue + (endValue - startValue) * progress;
            m_handPoseOutputPorts[i]->setValue(value);
            m_handPoseOutputPorts[i]->setValid(true);
        }

        // calculate the new state
        if (++m_stateProgress >= m_stateDurations[m_state])
        {
            m_stateProgress = 0;
            m_state++;
            if (m_state > 6)
            {
                m_state = 0;
            }
        }
    }
};
