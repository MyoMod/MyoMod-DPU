/**
 * @file port.h
 * @author Leon Farchau (leon2225)
 * @brief A Port describes a single input or output channel of a node. It is used to link a node to a channel of another node.
 * @version 0.1
 * @date 19.06.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include <memory>
#include <vector>
#include <span>


class LogNode;

class BaseOutputPort
{
public:
    friend class LogNode;

    virtual ~BaseOutputPort() = default;
    BaseOutputPort(BaseOutputPort&&) = default;
    BaseOutputPort& operator=(BaseOutputPort&&) = default;

    void setValid(bool valid) {m_valid = valid;};
    bool isValid() const {return m_valid;};

    void setError(bool error) {m_error = error;};
    bool hasError() const {return m_error;};

protected:
        // Interfaces for loggers
    virtual std::span<const std::byte> getRawData() const = 0;
    virtual std::string getDataType() const = 0;

    BaseOutputPort() = default;
    BaseOutputPort(const BaseOutputPort&) = delete;
    BaseOutputPort& operator=(const BaseOutputPort&) = delete;

private:
    bool m_valid = true;
    bool m_error = false;
};

template <typename T>
class OutputPort : public BaseOutputPort
{
public:
    void setValue(T const & value) {m_value = value;};
    const T& getValue() const {return m_value;};

    std::span<const std::byte> getRawData() const override
    {
        const std::byte * ptr = reinterpret_cast<const std::byte *>(&m_value);
        std::span<const std::byte> span(ptr, sizeof(T));
        return std::span<const std::byte>(reinterpret_cast<const std::byte *>(&m_value), sizeof(T));
        //return std::span (reinterpret_cast<std::byte const *>(&m_value), sizeof(T));
        //return std::span<std::byte>(reinterpret_cast<std::byte const *>(&m_value), reinterpret_cast<std::byte const *>(&m_value + 1));
    }
    std::string getDataType() const override
    {
        return typeid(T).name();
    }
private:
    T m_value;
};



class BaseInputPort
{
public:
    virtual ~BaseInputPort() = default;

    virtual bool linkTo(std::shared_ptr<BaseOutputPort> outputPort) = 0;
};

template <typename T>
class InputPort : public BaseInputPort
{
public:
    InputPort() = default;
    InputPort(T defaultValue):
        m_defaultValue{defaultValue},
        m_linkedOutputPort{}
        {}
    ~InputPort() = default;

    bool isValid() const {
        if (!m_linkedOutputPort)
        {
            return false;
        }
        return m_linkedOutputPort->isValid();};
    bool hasError() const {
        if(!m_linkedOutputPort)
        {
            return true;
        }
        return m_linkedOutputPort->hasError();};

    /**
     * @brief Link the InputPort to an OutputPort. If the OutputPort is not of the correct type, the link is not established.
     * 
     * @param outputPort    The OutputPort to link to
     * @return true         If the link was established
     * @return false        If the link was not established
     */
    bool linkTo(std::shared_ptr<BaseOutputPort> outputPort) override{
        m_linkedOutputPort = std::dynamic_pointer_cast<OutputPort<T>>(outputPort);
        return !!m_linkedOutputPort;
    }
    bool isLinked() const {return !!m_linkedOutputPort;};    


    /**
     * @brief Get the Value of the linked OutputPort. If the OutputPort is not linked or has an error, the default value is returned.
     * 
     * @param value     The value of the OutputPort
     */
    const T& getValue() 
    {
        if (isValid())
        {
            return m_linkedOutputPort->getValue(); 
        } else 
        {
            return m_defaultValue;
        }
    }

private:
    T m_defaultValue;
    std::shared_ptr<OutputPort<T>>m_linkedOutputPort;
};