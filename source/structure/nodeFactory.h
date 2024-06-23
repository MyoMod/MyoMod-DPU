/**
 * @file nodeFactory.h
 * @author Leon Farchau (leon2225)
 * @brief The NodeFactory is used to create and parametrize nodes. 
 * @version 0.1
 * @date 20.06.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <memory>

#include "node.h"

enum class NodeType
{
    InputNode,
    FloatInputNode,
    ArrayInputNode,
    Float2IntNode,
    Int2FloatNode,
    AdditonNode,
    ArraySelectorNode,
    OutputNode,
    None
};


class NodeFactory
{
public:
    template <typename... Args>
    static std::unique_ptr<AlgorithmicNode> createAlgorithmicNode(NodeType type, Args... args);

    template <typename... Args>
    static std::unique_ptr<DeviceNode> createDeviceNode(NodeType type, Args... args);
};

template <typename... Args>
std::unique_ptr<AlgorithmicNode> NodeFactory::createAlgorithmicNode(NodeType type, Args... args)
{
    switch (type)
    {
    case NodeType::Float2IntNode:
        return std::make_unique<CastNode<float, int>>();
    case NodeType::Int2FloatNode:
        return std::make_unique<CastNode<int, float>>();
    case NodeType::AdditonNode:
        return std::make_unique<AdditonNode>();
    case NodeType::ArraySelectorNode:
        return std::make_unique<ArraySelectorNode>(args...);
    default:
        return nullptr;
    }
}

template <typename... Args>
std::unique_ptr<DeviceNode> NodeFactory::createDeviceNode(NodeType type, Args... args)
{
    switch (type)
    {
    case NodeType::InputNode:
        return std::make_unique<InputNode>(args...);
    case NodeType::FloatInputNode:
        return std::make_unique<FloatInputNode>(args...);
    case NodeType::ArrayInputNode:
        return std::make_unique<ArrayInputNode>(args...);
    case NodeType::OutputNode:
        return std::make_unique<OutputNode>(args...);
    default:
        return nullptr;
    }
}