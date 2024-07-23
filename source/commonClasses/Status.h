#pragma once


// Enum for status of the program
enum class Status
{
    Ok = 1,
    Error,
    IllegalArgument,
    RegisterInvalid,
    RegisterNotImplemented,
    OutOfRange,
    Warning,
    Timeout
};
